#!/usr/bin/env python3
"""Nome vecchio di `f1t exp index` (tools/core/analysis/index.py), tenuto per tools/core 1.x."""
import os
import sys

print('tools/analysis_index.py: deprecato, usa `f1t exp index` (tools/core/analysis/index.py)', file=sys.stderr)
here = os.path.dirname(os.path.abspath(__file__))
target = os.path.join(here, 'core', 'analysis', 'index.py')
os.execv(sys.executable, [sys.executable, target, *sys.argv[1:]])
