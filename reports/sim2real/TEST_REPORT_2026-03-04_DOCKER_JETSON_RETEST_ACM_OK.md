# Sim2Real Jetson Tests - Docker Retest (ttyACM0 Passed Through)

Date: 2026-03-04
Environment: Jetson Nano (aarch64), Docker, Ubuntu 20.04, root

## Preconditions

- `/dev/ttyACM0` exists in container
- `pyserial` can discover `STM32 Virtual ComPort`

## Test Results

### Step 1

Command:

```bash
python3 01_env_check.py --port /dev/ttyACM0
```

Result:
- PASS: Python/dependencies all good
- PASS: serial device exists and has read/write permission

### Step 2

Command:

```bash
python3 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
```

Result:
- PASS: serial connected
- PASS: IMU values continuously observed
- Note: `servo_count=0` during this run (only IMU updates observed)

### Step 3 (dry-run)

Command:

```bash
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50
```

Result:
- PASS: dry-run entered
- PASS: command plan generated (`current=2048 -> target=2098 -> back=2048`)

### Step 4

Command:

```bash
python3 04_onnx_latency_smoke_test.py --iters 200
```

Result:
- PASS: provider `CPUExecutionProvider`
- PASS: output shape `(1, 12)`
- PASS: latency budget for 50Hz control
  - infer mean `0.1949 ms`, p95 `0.2349 ms`, p99 `0.2996 ms`
  - full mean `0.3116 ms`, p95 `0.4183 ms`, p99 `0.4781 ms`

## Conclusion

Current dockerized Jetson test flow is functional for:
- environment/dependency readiness,
- serial link probing,
- safe dry-run command path,
- ONNX inference latency smoke test.

If mechanical safety is confirmed, next step is real motion test:

```bash
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50 --execute
```
