# CLAUDE.md

Working notes for this workspace. Not a full conventions document yet — seeded
with the gotchas that have already cost real debugging time. Add to it when
something bites.

## Testing

### `colcon test` runs nothing unless `setup.py` declares a `test` extra

colcon's `ament_python` test step only invokes pytest for a package whose
`setup.py` declares:

```python
extras_require={
    'test': ['pytest'],
},
```

Without it, colcon falls back to `setup.py test`, whose unittest discovery
finds no pytest-style tests and prints:

```
Ran 0 tests in 0.000s

OK
```

That is a **green result that executed nothing**. Until 2026-09-03 only
`mpc_controller` and `llm` had the extra, so every other package's test suite
had never run — 628 tests across 17 packages were dead weight. Fixed workspace-wide
in `aaae377`.

`tests_require=['pytest']` is **not** the same thing and does nothing: modern
setuptools does not recognize it, warns `UserWarning: Unknown distribution
option: 'tests_require'` on every build, and ignores it. Removed in `626fb7b`.

**When adding a new ament_python package, declare the `test` extra.** If a
package reports suspiciously few tests, check its `setup.py` before believing it.

### A test that never runs will contradict the code and nobody notices

`f1tenth_perception/test/test_detection_launch_config.py` asserted that
`yolo_model` defaulted to `yolo26s.engine`. The same commit (`96c6bbc`) changed
that default to `yolo26s-seg.pt`. The test was wrong from the moment it was
written and survived two days of live runs on the new default, because colcon
never executed the file (see above). It surfaced the instant real test discovery
was switched on.

Two habits that would have caught it:

- **Name a test after what it asserts.** The stale one was called
  `test_default_launch_stays_on_tensorrt_box_detector_unchanged` while the
  deployed default had moved to segmentation — the name actively argued for
  the wrong behaviour.
- **When a test encodes a config default, it is a coupling.** Changing the
  default in `stack_params.yaml` means updating the test in the same commit.

## Packaging

### Never put a literal `--` inside a `package.xml` XML comment

`--` is illegal inside an XML comment body. colcon does not fail loudly; it
silently downgrades the package type to plain `python`, which then shows up as
a runtime-only "package not found" crash long after the build reported success.
Use a single hyphen in comment prose.
