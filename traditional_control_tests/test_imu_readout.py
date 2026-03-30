"""Compatibility wrapper for migrated script."""
from pathlib import Path
import runpy
import sys

if __name__ == "__main__":
    repo_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root))
    runpy.run_module("common.tools.imu.test_imu_readout", run_name="__main__")
