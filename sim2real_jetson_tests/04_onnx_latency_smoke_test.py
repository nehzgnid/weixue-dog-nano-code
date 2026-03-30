"""Compatibility wrapper for migrated script."""
import runpy
from pathlib import Path

if __name__ == "__main__":
    target = Path(__file__).resolve().parents[1] / "rl" / "tests" / "smoke" / "04_onnx_latency_smoke_test.py"
    runpy.run_path(str(target), run_name="__main__")
