#!/usr/bin/env python3
"""Reject a parameter set that is not inside tuning/space.yaml.  check_params.py next_params.yaml"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import common  # noqa: E402


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    params = common.load_yaml(argv[0]) or {}
    if not isinstance(params, dict):
        print(f'{argv[0]}: not a mapping', file=sys.stderr)
        return 1
    problems = common.check_params(params, common.load_space())
    for p in problems:
        print(p, file=sys.stderr)
    if not problems:
        print(f'{argv[0]}: ok ({len(params)} parameters, the rest at their defaults)')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
