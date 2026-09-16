"""Make the repo root importable so ``go_to_object`` resolves without install."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
