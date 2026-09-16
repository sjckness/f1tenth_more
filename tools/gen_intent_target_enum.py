#!/usr/bin/env python3
"""Regenerate the ``go_to.target`` enum in intent_v1.json from the detector.

The class vocabulary is NOT defined anywhere in this repo as code. It lives
inside the YOLO checkpoint and is read at runtime by
yolo_detector_node._resolve_class_names(), so which strings exist depends on
stack_params.yaml's `yolo_model`. The two checkpoints currently in the tree
share only 10 of their classes:

    yolo26s-seg.pt   80 COCO classes
    yolo26_office.pt 58 custom office classes, 48 of them not in COCO

So a static enum in the schema is a snapshot of one model, and switching
`yolo_model` silently invalidates it. This script regenerates the snapshot,
and test_intent_go_to.py fails when the schema and the configured checkpoint
disagree -- which is the closest thing to "cannot drift" available while the
vocabulary lives in a binary.

Usage:  python3 tools/gen_intent_target_enum.py [--check]
"""

import argparse
import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
SCHEMA = REPO / 'src/f1tenth_intelligence/llm/schemas/intent_v1.json'
PARAMS = REPO / 'src/f1tenth_params/config/stack_params.yaml'
MODELS = REPO / 'src/f1tenth_perception/models'


def configured_model_name() -> str:
    """The `yolo_model` default from stack_params.yaml, without pulling yaml."""
    lines = PARAMS.read_text().splitlines()
    for i, line in enumerate(lines):
        if line.startswith('yolo_model:'):
            for follow in lines[i + 1:i + 4]:
                if 'default:' in follow:
                    return follow.split('default:', 1)[1].strip()
    raise SystemExit('could not find yolo_model.default in stack_params.yaml')


def class_names(checkpoint: pathlib.Path) -> list[str]:
    import torch
    blob = torch.load(checkpoint, map_location='cpu', weights_only=False)
    names = blob['model'].names
    return sorted(str(n) for n in names.values())


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', action='store_true',
                        help='exit non-zero on drift instead of rewriting')
    args = parser.parse_args(argv)

    model = configured_model_name()
    checkpoint = MODELS / model
    if not checkpoint.exists():
        raise SystemExit(f'configured model {model!r} not found at {checkpoint}')
    if checkpoint.suffix != '.pt':
        raise SystemExit(
            f'{model!r} is not a .pt checkpoint; class names for a .engine build '
            'come from a sibling .pt (see yolo_detector_node._resolve_class_names)')

    names = class_names(checkpoint)
    schema = json.loads(SCHEMA.read_text())
    branches = schema['properties']['plan']['items']['oneOf']
    target = next(b for b in branches
                  if b['properties']['mode'].get('const') == 'go_to')
    current = target['properties']['target']['enum']

    if current == names:
        print(f'up to date: {len(names)} classes from {model}')
        return 0
    if args.check:
        print(f'DRIFT: schema has {len(current)} classes, {model} has {len(names)}')
        print(f'  only in schema: {sorted(set(current) - set(names))[:10]}')
        print(f'  only in model : {sorted(set(names) - set(current))[:10]}')
        return 1

    target['properties']['target']['enum'] = names
    SCHEMA.write_text(json.dumps(schema, indent=2, ensure_ascii=False) + '\n')
    print(f'wrote {len(names)} classes from {model} into {SCHEMA.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
