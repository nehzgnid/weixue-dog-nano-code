"""Legacy backup wrapper for old module path: robot_config.py."""
from __future__ import annotations

import sys
from pathlib import Path

def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "MIGRATION_PLAN_SCHEME_B_EXECUTION.md").exists():
            return parent
    raise RuntimeError("Could not locate repository root from legacy backup wrapper")


_REPO_ROOT = _repo_root()
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from common.config.robot_config import *  # noqa: F401,F403
