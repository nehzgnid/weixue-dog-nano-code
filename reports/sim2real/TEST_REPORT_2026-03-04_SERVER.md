# Sim2Real Jetson Tests - Server Validation Report

Date: 2026-03-04
Environment: server (x86_64), conda `env_isaaclab`
Scope: Validate script/dependency/inference pipeline; hardware checks expected to fail without `/dev/ttyACM0`.

## Commands and Results

### Step 1
Command:

```bash
python 01_env_check.py --port /dev/ttyACM0
```

Result:

- PASS: python, numpy, serial(pyserial), yaml, onnxruntime
- FAIL: `/dev/ttyACM0` not found

### Step 2
Command:

```bash
python 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 2
```

Result:

- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

### Step 3
Command:

```bash
python 03_single_servo_safe_test.py --id 1 --delta 50
```

Result:

- Entered dry-run mode correctly
- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

### Step 4
Command:

```bash
python 04_onnx_latency_smoke_test.py \
  --model /home/user/IsaacLab/sim2real_test/config/wavego_policy.onnx \
  --stats /home/user/IsaacLab/sim2real_test/config/normalizer_stats.npz \
  --iters 500
```

Result:

- PASS: output shape `(1, 12)`
- PASS: inference latency
  - infer mean `0.0250 ms`, p95 `0.0274 ms`, p99 `0.0375 ms`
  - full mean `0.0323 ms`, p95 `0.0373 ms`, p99 `0.0451 ms`

## Conclusion

This server run is useful and completed as intended:

1. Script integrity validated (runnable, correct dependency boundaries).
2. Inference path validated with real ONNX model.
3. Remaining blockers are hardware-only (`/dev/ttyACM0` unavailable), not software pipeline issues.

## Next Action on Jetson Nano

Repeat Step 1~3 on Jetson with STM32 connected:

```bash
cd /home/user/weixue-dog-nano-code/sim2real_jetson_tests
python 01_env_check.py --port /dev/ttyACM0
python 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
python 03_single_servo_safe_test.py --id 1 --delta 50 --execute
```

If Step 2/3 still fail on Jetson, troubleshoot USB CDC enumeration and dialout permission first.
