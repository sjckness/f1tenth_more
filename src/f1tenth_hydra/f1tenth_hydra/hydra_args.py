"""Build the Hydra node's command line from a profile and the launch options.

Pure Python (no launch objects), so hydra.launch.py stays a thin wrapper and the argument list
can be printed (`python3 -m f1tenth_hydra.hydra_args`) and checked without Hydra installed.
"""

import argparse
import datetime
import json
import os

from f1tenth_hydra.camera import config_dir, hydra_param_extrinsics, load_camera_contract
from f1tenth_hydra.camera import load_yaml

SEG_SOURCES = ('gt', 'mobilenet')
POSE_SOURCES = ('isaac_gt', 'stack_tf')
INPUT_NAME = 'zed'                 # Hydra input name -> /hydra/input/zed/...
HYDRA_NODE = 'hydra'               # -> /hydra/backend/dsg, /hydra/status
SEG_NS = 'hydra_seg'               # semantic_inference namespace (seg_source:=mobilenet)
GT_PARENT = 'map_sim'              # Isaac GT world (frame of /sim/ground_truth/odom)
GT_CHILD = 'hydra_gt_base_link'    # base_link pose from GT, a tree of its own
MAP_FRAME = 'hydra_map'            # never `map`: the stack owns map -> odom


def profile_path(name):
    return os.path.join(config_dir(), 'profiles', f'{name}.yaml')


def load_profile(name):
    path = profile_path(name)
    if not os.path.isfile(path):
        known = sorted(f[:-5] for f in os.listdir(os.path.dirname(path)) if f.endswith('.yaml'))
        raise ValueError(f'unknown hydra_profile "{name}" (known: {known})')
    prof = load_yaml(path)
    prof['_path'] = path
    return prof


def default_log_dir(profile_name):
    stamp = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
    return os.path.join(os.path.expanduser('~'), '.hydra', 'f1tenth', f'{profile_name}_{stamp}')


def _yaml_arg(obj):
    # JSON is YAML: one argv token, no shell quoting needed.
    return json.dumps(obj, separators=(',', ':'))


def frames(pose_source):
    if pose_source == 'isaac_gt':
        return {'map_frame': MAP_FRAME, 'odom_frame': GT_PARENT, 'robot_frame': GT_CHILD}
    return {'map_frame': MAP_FRAME, 'odom_frame': 'odom', 'robot_frame': 'base_link'}


def input_block(profile, pose_source, contract):
    sensor = profile['sensor']
    if pose_source == 'isaac_gt':
        # GT body pose + the deployed camera mount (the TF the stack publishes and Isaac renders).
        extrinsics = hydra_param_extrinsics(contract)
    else:
        # base_link -> camera_info header frame from the stack's /tf_static.
        extrinsics = {'type': 'ros'}
    return {'input': {
        'type': 'RosInput',
        'clear_queue_on_fail': True,
        'inputs': {INPUT_NAME: {
            'receiver': {'type': 'ClosedSetImageReceiver',
                         'queue_size': int(sensor.get('queue_size', 30))},
            'sensor': {'type': 'camera_info',
                       'min_range': float(sensor['min_range_m']),
                       'max_range': float(sensor['max_range_m']),
                       'extrinsics': extrinsics},
        }},
    }}


def remappings(seg_source, contract):
    t = contract['topics']
    base = f'/{HYDRA_NODE}/input/{INPUT_NAME}'
    semantic = t['semantic_gt'] if seg_source == 'gt' else f'/{SEG_NS}/semantic/image_raw'
    return [
        (f'{base}/rgb/image_raw', t['rgb']),
        (f'{base}/rgb/camera_info', t['rgb_info']),
        (f'{base}/depth_registered/image_rect', t['depth']),
        (f'{base}/semantic/image_raw', semantic),
    ]


def build(profile, seg_source, pose_source, log_dir, hydra_share, hydra_ros_share,
          label_remap_path=None, extra_yaml=None, contract=None):
    """Return {'arguments': [...], 'remappings': [...], 'frames': {...}, ...} for the node."""
    if seg_source not in SEG_SOURCES:
        raise ValueError(f'seg_source must be one of {SEG_SOURCES}, got "{seg_source}"')
    if pose_source not in POSE_SOURCES:
        raise ValueError(f'pose_source must be one of {POSE_SOURCES}, got "{pose_source}"')
    if seg_source == 'gt' and not label_remap_path:
        raise ValueError('seg_source:=gt needs the Isaac -> Hydra label remap file')
    contract = contract or load_camera_contract()
    seg = profile['segmentation']
    fr = frames(pose_source)
    lcd = bool(profile.get('lcd_with_stack_tf', False)) and pose_source == 'stack_tf'
    cfg = os.path.join(config_dir(), profile['hydra_config'])
    args = [
        '--config-utilities-file', cfg,
        '--config-utilities-file',
        os.path.join(hydra_share, 'config', 'label_spaces', f'{seg["labelspace"]}_label_space.yaml'),
    ]
    if profile.get('viz_sinks'):
        sinks = os.path.join(hydra_ros_share, 'config', 'sinks')
        args += ['--config-utilities-file', os.path.join(sinks, 'mesh_segmenter_sinks.yaml')
                 + '@frontend/objects',
                 '--config-utilities-file', os.path.join(sinks, 'gvd_places_sinks.yaml')
                 + '@frontend/freespace_places',
                 '--config-utilities-file', os.path.join(sinks, 'active_window_sinks.yaml')
                 + '@active_window']
    args += ['--config-utilities-yaml', _yaml_arg({'enable_lcd': lcd})]
    if lcd:
        args += ['--config-utilities-file', os.path.join(hydra_share, 'config', 'lcd', 'default.yaml')]
    args += [
        '--config-utilities-yaml', _yaml_arg(input_block(profile, pose_source, contract)),
        '--config-utilities-yaml', _yaml_arg(dict(fr, robot_id=0)),
        '--config-utilities-yaml', _yaml_arg({'log_path': log_dir, 'exit_after_clock': False}),
        '--config-utilities-yaml', _yaml_arg({
            'glog_level': int(profile.get('glog_level', 0)),
            'glog_verbosity': int(profile.get('glog_verbosity', 0))}),
    ]
    if pose_source == 'stack_tf':
        # Hydra's map -> odom would give the stack's `odom` a second parent.
        args += ['--config-utilities-yaml', _yaml_arg({'backend': {'publish_backend_tf': False}})]
    if seg_source == 'gt':
        args += ['--config-utilities-yaml',
                 _yaml_arg({'semantic_label_remap_filepath': label_remap_path})]
    if extra_yaml and extra_yaml.strip() not in ('', '{}'):
        args += ['--config-utilities-yaml', extra_yaml]
    return {
        'arguments': args,
        'remappings': remappings(seg_source, contract),
        'frames': fr,
        'enable_lcd': lcd,
        'log_dir': log_dir,
        'segmentation': seg,
    }


def main():
    """Print the Hydra command line for a profile (no ROS, no Hydra needed)."""
    p = argparse.ArgumentParser(description=main.__doc__)
    p.add_argument('--profile', default='workstation_full')
    p.add_argument('--seg-source', default=None)
    p.add_argument('--pose-source', default='isaac_gt')
    a = p.parse_args()
    prof = load_profile(a.profile)
    seg = a.seg_source or prof['default_seg_source']
    out = build(prof, seg, a.pose_source, default_log_dir(a.profile), '<hydra share>',
                '<hydra_ros share>', label_remap_path='<remap.yaml>')
    print('ros2 run hydra_ros hydra_ros_node \\')
    for i in range(0, len(out['arguments']), 2):
        print(f"  {out['arguments'][i]} '{out['arguments'][i + 1]}' \\")
    for src, dst in out['remappings']:
        print(f'  --ros-args -r {src}:={dst}')


if __name__ == '__main__':
    main()
