"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("common.tools.latency.test_latency_linux", run_name="__main__")
