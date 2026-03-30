"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("traditional.apps.control_center_unified", run_name="__main__")
