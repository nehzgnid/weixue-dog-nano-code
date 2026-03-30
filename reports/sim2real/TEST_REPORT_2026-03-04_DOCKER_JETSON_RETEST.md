# Sim2Real Jetson Tests - Docker Retest Report

Date: 2026-03-04
Environment: Jetson Nano (aarch64), Docker, Ubuntu 20.04, root

## Change Before Retest

Updated `04_onnx_latency_smoke_test.py` default paths:

- model: `/workspace/weixue-dog-nano-code/sim2real_jetson_tests/config/wavego_policy.onnx`
- stats: `/workspace/weixue-dog-nano-code/sim2real_jetson_tests/config/normalizer_stats.npz`

## Retest Commands and Results

### Step 1

```bash
python3 01_env_check.py --port /dev/ttyACM0
```

Result:
- PASS: Python + dependencies (`numpy`, `serial`, `yaml`, `onnxruntime`)
- FAIL: `/dev/ttyACM0` not found

### Step 2

```bash
python3 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
```

Result:
- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

### Step 3 (dry-run)

```bash
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50
```

Result:
- PASS: dry-run mode entered
- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

### Step 4 (updated default paths)

```bash
python3 04_onnx_latency_smoke_test.py --iters 200
```

Result:
- PASS: provider `CPUExecutionProvider`
- PASS: output shape `(1, 12)`
- PASS: latency budget
  - infer mean `0.1593 ms` (p95 `0.2048 ms`, p99 `0.2681 ms`)
  - full mean `0.2666 ms` (p95 `0.3613 ms`, p99 `0.4719 ms`)

## Conclusion

- Path update is correct and effective for ONNX smoke test.
- Remaining blocker is still Docker serial device mapping (`/dev/ttyACM0` absent).
