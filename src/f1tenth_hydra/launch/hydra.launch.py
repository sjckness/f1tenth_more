"""Hydra 3D scene graph on the simulated (or real) ZED 2i.

    ros2 launch f1tenth_hydra hydra.launch.py [hydra_profile:=workstation_full]
        [seg_source:=gt|mobilenet] [pose_source:=isaac_gt|stack_tf] [use_sim_time:=true]

Started on its own or by scripts/sim/sim_isaac.sh with hydra:=true (docs/hydra_hil.md). It is
NOT part of supervisor_bringup: no stack default changes with this file.

- hydra_profile: config/profiles/<name>.yaml (workstation_full, jetson_min).
- seg_source: gt = Isaac GT classes (/sim/ground_truth/left/semantic), remapped to the Hydra
  label space by config/labels/isaac_to_ade20k_mp3d.yaml; mobilenet = MobileNetV2 (ADE20K) via
  semantic_inference's TensorRT closed_set_node (engine built on first run on this GPU).
  Default: the profile's default_seg_source.
- pose_source: isaac_gt = /sim/ground_truth/odom turned into TF map_sim -> hydra_gt_base_link
  (hydra_ros odom_to_tf) + the deployed camera mount; stack_tf = the stack's odom -> base_link
  and base_link -> camera TF (EKF drift included; loop closures on if the profile allows).
- scene_semantics: an Isaac scene yaml whose `semantics:` ids the GT labels use (default: the
  base_room table, which office/hospital share).

Hydra itself comes from its own workspace (~/hydra_ws): source it before this workspace.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, Shutdown)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def _truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _share(pkg):
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def _setup(context):
    from f1tenth_hydra import hydra_args, labels
    from f1tenth_hydra.camera import config_dir, load_camera_contract

    lc = {k: LaunchConfiguration(k).perform(context) for k in (
        'hydra_profile', 'seg_source', 'pose_source', 'use_sim_time', 'scene_semantics',
        'log_dir', 'visualizer', 'rviz', 'rviz_config', 'extra_yaml')}
    profile = hydra_args.load_profile(lc['hydra_profile'])
    seg_source = lc['seg_source'] or profile['default_seg_source']
    pose_source = lc['pose_source']
    use_sim_time = _truthy(lc['use_sim_time'])
    log_dir = os.path.expanduser(lc['log_dir']) if lc['log_dir'] else \
        hydra_args.default_log_dir(profile['name'])
    os.makedirs(log_dir, exist_ok=True)

    hydra_share, hydra_ros_share = _share('hydra'), _share('hydra_ros')
    if not hydra_share or not hydra_ros_share:
        return [LogInfo(msg='hydra.launch.py: packages hydra / hydra_ros not found. Build '
                            '~/hydra_ws (scripts/hydra/build_hydra_ws.sh) and source '
                            '~/hydra_ws/install/setup.bash BEFORE this workspace.'),
                Shutdown(reason='Hydra not installed')]

    contract = load_camera_contract()
    actions = []
    remap_path = None
    if seg_source == 'gt':
        remap_path, missing = labels.write_remap(
            os.path.join(log_dir, 'isaac_label_remap.yaml'),
            scene_file=lc['scene_semantics'] or None)
        if missing:
            actions.append(LogInfo(msg=f'hydra: Isaac classes with no Hydra label (dropped): '
                                       f'{missing} -- add them to {config_dir()}/labels/'))

    out = hydra_args.build(profile, seg_source, pose_source, log_dir, hydra_share,
                           hydra_ros_share, label_remap_path=remap_path,
                           extra_yaml=lc['extra_yaml'], contract=contract)
    params = [{'use_sim_time': use_sim_time}]

    if pose_source == 'isaac_gt':
        actions.append(Node(
            package='hydra_ros', executable='odom_to_tf', name='hydra_gt_odom_to_tf',
            parameters=params + [{'parent_frame': hydra_args.GT_PARENT,
                                  'child_frame': hydra_args.GT_CHILD}],
            remappings=[('/hydra_gt_odom_to_tf/odom', contract['topics']['gt_odom'])],
            output='log'))
    else:
        # Hydra publishes no TF here (the stack owns odom's parent), so anchor hydra_map at odom:
        # the graph shows in RViz under hydra_map, map or odom. Hydra's own loop-closure
        # correction (map_T_odom) is not shown by this anchor.
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='hydra_map_anchor', parameters=params,
            arguments=['--frame-id', 'odom', '--child-frame-id', hydra_args.MAP_FRAME],
            output='log'))

    if seg_source == 'mobilenet':
        si_share = _share('semantic_inference_ros')
        if not si_share:
            return [LogInfo(msg='hydra.launch.py: seg_source:=mobilenet needs '
                                'semantic_inference_ros (part of ~/hydra_ws).'),
                    Shutdown(reason='semantic_inference_ros not installed')]
        seg = out['segmentation']
        actions.append(GroupAction([
            PushRosNamespace(hydra_args.SEG_NS),
            SetRemap(src=f'/{hydra_args.SEG_NS}/color/image_raw', dst=contract['topics']['rgb']),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(si_share, 'launch', 'closed_set.launch.yaml')),
                launch_arguments={
                    'model_name': seg['model_name'],
                    'labelspace_name': seg['labelspace'],
                    'min_separation_s': str(seg.get('min_separation_s', 0.0)),
                    'max_image_queue_size': '1',
                    'show_configs': 'false',
                }.items()),
        ]))

    actions.append(Node(
        package='hydra_ros', executable='hydra_ros_node', name=hydra_args.HYDRA_NODE,
        arguments=out['arguments'], remappings=out['remappings'], parameters=params,
        output='screen', on_exit=Shutdown(reason='hydra_ros_node exited')))

    visualizer = _truthy(lc['visualizer']) if lc['visualizer'] else bool(profile.get('visualizer'))
    rviz = _truthy(lc['rviz']) if lc['rviz'] else bool(profile.get('rviz'))
    if visualizer:
        viz_share = _share('hydra_visualizer')
        rviz_cfg = lc['rviz_config'] or os.path.join(
            _share('f1tenth_hydra'), 'rviz', 'hydra_hil.rviz')
        actions.append(GroupAction([
            SetRemap(src='/hydra_visualizer/dsg', dst=f'/{hydra_args.HYDRA_NODE}/backend/dsg'),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(viz_share, 'launch', 'streaming_visualizer.launch.yaml')),
                launch_arguments={
                    'visualizer_frame': hydra_args.MAP_FRAME,
                    'external_plugins_path': os.path.join(
                        hydra_ros_share, 'config', 'hydra_ros_visualizer_plugins.yaml'),
                    'start_rviz': 'true' if rviz else 'false',
                    'rviz_path': rviz_cfg,
                }.items()),
        ]))

    actions.insert(0, LogInfo(msg=(
        f'hydra: profile {profile["name"]}, seg_source {seg_source}, pose_source {pose_source}, '
        f'frames {out["frames"]}, lcd {out["enable_lcd"]}, visualizer {visualizer} (rviz {rviz}), '
        f'log {log_dir}')))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('hydra_profile', default_value='workstation_full',
                              description='config/profiles/<name>.yaml'),
        DeclareLaunchArgument('seg_source', default_value='',
                              description="gt | mobilenet ('' = the profile's default)"),
        DeclareLaunchArgument('pose_source', default_value='isaac_gt',
                              description='isaac_gt | stack_tf'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scene_semantics', default_value='',
                              description='Isaac scene yaml for the GT class ids '
                                          "('' = base_room table)"),
        DeclareLaunchArgument('log_dir', default_value='',
                              description="Hydra log/output dir ('' = ~/.hydra/f1tenth/"
                                          '<profile>_<stamp>)'),
        DeclareLaunchArgument('visualizer', default_value='',
                              description="hydra_visualizer ('' = profile)"),
        DeclareLaunchArgument('rviz', default_value='', description="RViz ('' = profile)"),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description="RViz config ('' = rviz/hydra_hil.rviz)"),
        DeclareLaunchArgument('extra_yaml', default_value='{}',
                              description='extra config_utilities yaml for Hydra (last wins)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
