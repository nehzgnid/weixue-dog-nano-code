"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("traditional.tests.test_single_leg_step", run_name="__main__")
