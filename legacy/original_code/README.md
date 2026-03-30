# Legacy Backup (Runnable)

Generated at: 2026-03-30 13:12:37 UTC

This folder keeps runnable backup wrappers for pre-migration paths.
Each wrapper auto-locates the repository root and forwards execution to the migrated implementation.

## Quick Usage

```bash
cd /workspace/weixue-dog-nano-code
python3 legacy/original_code/main_trot.py --help
python3 legacy/original_code/traditional_control_tests/test_imu_readout.py --help
python3 legacy/original_code/sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py --help
```

## Notes

- This is a runnable backup for old paths, not the primary runtime path.
- For hardware-related scripts, run --help or dry-run first.
