"""
Entry point: the Telegram bot, or the same panel in a terminal (--cli).

    python3 -m f1tenth_telegram [--config FILE]            the bot (needs the token)
    python3 -m f1tenth_telegram --cli [--config FILE]      interactive, no Telegram
    python3 -m f1tenth_telegram --cli --do connect --do start:stack --do confirm ...
"""

import argparse
import logging
import sys

from . import cli, config
from .controller import Controller
from .transport import Transport


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog='f1tenth_telegram', description=__doc__.split('\n\n')[0])
    p.add_argument('--config', help='YAML config (default: $F1T_TELEGRAM_CONFIG or '
                                    'config/telegram.yaml)')
    p.add_argument('--cli', action='store_true', help='terminal panel, no Telegram')
    p.add_argument('--do', action='append', metavar='ACTION',
                   help='scripted --cli: run these actions in order')
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
    try:
        cfg = config.load(args.config)
    except config.ConfigError as exc:
        print(f'f1tenth_telegram: {exc}', file=sys.stderr)
        return 2
    controller = Controller(cfg, Transport(cfg.target))
    if args.cli or args.do:
        return cli.run(controller, args.do)
    try:
        token = config.read_token(cfg)
    except config.ConfigError as exc:
        print(f'f1tenth_telegram: {exc}', file=sys.stderr)
        return 2
    try:
        from . import tg
        tg.run_bot(cfg, controller, token)
    except ImportError as exc:
        print(f'f1tenth_telegram: python-telegram-bot is not installed ({exc}); '
              'see README.md, or use --cli', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
