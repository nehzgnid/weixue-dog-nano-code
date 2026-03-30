"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("traditional.tests.test_trot_in_place", run_name="__main__")
