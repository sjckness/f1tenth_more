"""
Run the gateway: f1tenth_gateway --config config/gateway.yaml [--mock] [--port N].

On the car bin/f1tenth_gateway starts this with ctl.env sourced (ROS,
discovery server, the workspace) and the venv's Python. --mock runs it on a
pretend car (mock.py): no ROS, no f1tenth_ctl, for linus and the tests.
"""

import argparse
import logging
import sys

from . import config


def build(cfg, mock: bool):
    from .app import Gateway, create_app
    token = config.token(cfg)
    if mock or cfg['mock']:
        from .mock import MockCtl, MockRos, MockWorld
        from .transcription import FakeTranscriber
        world = MockWorld()
        ctl = MockCtl(world)

        def make_ros(teleop, spec, on_status, on_outcome):
            return MockRos(world, teleop, on_status, on_outcome)
        gw = Gateway(cfg, make_ros, ctl, FakeTranscriber(cfg['transcription']), token,
                     world=world)
    else:
        from .ctl_port import CtlPort
        from .ros_bridge import RosBridge
        from .transcription import TranscriberProcess
        ctl = CtlPort(cfg['ctl_env'])

        def make_ros(teleop, spec, on_status, on_outcome):
            return RosBridge(cfg, teleop, spec, on_status, on_outcome)
        gw = Gateway(cfg, make_ros, ctl, TranscriberProcess(cfg['transcription']), token)
    return gw, create_app(gw)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='f1tenth_gateway')
    ap.add_argument('--config', default=None)
    ap.add_argument('--mock', action='store_true', help='pretend car, no ROS (linus, tests)')
    ap.add_argument('--host', default=None)
    ap.add_argument('--port', type=int, default=None)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    over = {}
    if args.host or args.port:
        over['listen'] = {k: v for k, v in (('host', args.host), ('port', args.port)) if v}
    try:
        cfg = config.load(args.config, over or None)
        _gw, app = build(cfg, args.mock)
    except config.ConfigError as exc:
        print(f'f1tenth_gateway: {exc}', file=sys.stderr)
        return 2
    import uvicorn
    host, port = cfg['listen']['host'], cfg['listen']['port']
    logging.getLogger('f1tenth_gateway').info(
        'listening on http://%s:%d (%s); PWA from %s', host, port,
        'MOCK' if (args.mock or cfg['mock']) else 'car', cfg['www_dir'])
    uvicorn.run(app, host=host, port=port, log_level='warning', ws_ping_interval=None)
    return 0


if __name__ == '__main__':
    sys.exit(main())
