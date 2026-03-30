"""Compatibility wrapper for migrated script."""
import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    repo_root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(repo_root))
    runpy.run_module("rl.apps.wavego_keyboard_control", run_name="__main__")
