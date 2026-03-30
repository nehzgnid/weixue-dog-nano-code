"""Legacy backup wrapper for old script path: sim2real_jetson_tests/strategy_deploy_bundle/scripts/test_latency.py."""
from __future__ import annotations

import runpy
import sys
from pathlib import Path

def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            return parent
    raise RuntimeError("Could not locate repository root from legacy backup wrapper")


if __name__ == "__main__":
    repo_root = _repo_root()
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    target = repo_root / "rl/tools/test_latency.py"
    runpy.run_path(str(target), run_name="__main__")
