"""Compatibility wrapper for migrated script."""
import runpy

if __name__ == "__main__":
    runpy.run_module("traditional.apps.servo_control_center", run_name="__main__")
