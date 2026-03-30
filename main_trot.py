"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("traditional.apps.main_trot", run_name="__main__")
