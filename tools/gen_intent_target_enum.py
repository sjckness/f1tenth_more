#!/usr/bin/env python3
"""Regenerate the object-class snapshots from the detector.

Two snapshots of the same vocabulary, written from one checkpoint read:

  * the ``go_to.target`` enum in llm/schemas/intent_v1.json (the planner's
    validator), and
  * f1tenth_behavior/mission/object_classes.py, which mission_config.py
    validates a go_to_object move's target_class against. A copy rather than
    a read of the schema: llm exec-depends on f1tenth_behavior, so the
    behaviour package reading llm's share directory would be a cycle.

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
BEHAVIOR_CLASSES = REPO / 'src/f1tenth_behavior/f1tenth_behavior/mission/object_classes.py'
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


def render_behavior_module(model: str, names: list[str]) -> str:
    lines = [
        '"""Object classes a go_to_object move may target. GENERATED -- do not edit.',
        '',
        'Written by tools/gen_intent_target_enum.py from the configured detector',
        f'checkpoint ({model}), alongside the go_to.target enum in llm\'s',
        'intent_v1.json. Regenerate with that script after changing yolo_model;',
        'test_go_to_object_schema.py fails when this and the schema disagree.',
        '"""',
        '',
        f'SOURCE_MODEL = {model!r}',
        '',
        'OBJECT_CLASSES = frozenset({',
    ]
    lines += [f'    {name!r},' for name in names]
    lines.append('})')
    return '\n'.join(lines) + '\n'


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
    behavior_text = render_behavior_module(model, names)
    behavior_current = (BEHAVIOR_CLASSES.read_text()
                        if BEHAVIOR_CLASSES.exists() else None)

    schema_ok = current == names
    behavior_ok = behavior_current == behavior_text
    if schema_ok and behavior_ok:
        print(f'up to date: {len(names)} classes from {model}')
        return 0
    if args.check:
        if not schema_ok:
            print(f'DRIFT: schema has {len(current)} classes, {model} has {len(names)}')
            print(f'  only in schema: {sorted(set(current) - set(names))[:10]}')
            print(f'  only in model : {sorted(set(names) - set(current))[:10]}')
        if not behavior_ok:
            print(f'DRIFT: {BEHAVIOR_CLASSES.name} does not match {model}')
        return 1

    if not schema_ok:
        target['properties']['target']['enum'] = names
        SCHEMA.write_text(json.dumps(schema, indent=2, ensure_ascii=False) + '\n')
        print(f'wrote {len(names)} classes from {model} into {SCHEMA.name}')
    if not behavior_ok:
        BEHAVIOR_CLASSES.write_text(behavior_text)
        print(f'wrote {len(names)} classes from {model} into {BEHAVIOR_CLASSES.name}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
