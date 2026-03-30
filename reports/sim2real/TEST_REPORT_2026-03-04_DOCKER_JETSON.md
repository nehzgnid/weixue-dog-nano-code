# Sim2Real Jetson Tests - Docker Runtime Report

Date: 2026-03-04
Environment: Jetson Nano (aarch64), Docker, Ubuntu 20.04, no conda, running as root
Scope: Execute `sim2real_jetson_tests` in-container and identify blockers

## Step 1: Environment Check
Command:

```bash
python3 01_env_check.py --port /dev/ttyACM0
```

Result:

- PASS: Python 3.8.10
- PASS: `numpy` 1.24.4
- PASS: `serial` 3.5
- PASS: `yaml` 6.0.3
- PASS: `onnxruntime` 1.16.3
- FAIL: `/dev/ttyACM0` not found

## Step 2: STM32 State Probe
Command:

```bash
python3 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
```

Result:

- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

## Step 3: Single Servo Safe Test (dry-run)
Command:

```bash
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50
```

Result:

- PASS: entered dry-run mode
- FAIL: serial open failed (`No such file or directory: /dev/ttyACM0`)

## Step 4: ONNX Latency Smoke Test
Command:

```bash
python3 04_onnx_latency_smoke_test.py --iters 200
```

Result:

- FAIL: default stats/model path not found
  - `/home/user/IsaacLab/sim2real_test/config/normalizer_stats.npz`
  - `/home/user/IsaacLab/sim2real_test/config/wavego_policy.onnx`
- Additional workspace scan: no `.onnx` / `.npz` files found under this repository

## Extra Diagnostics (inside container)

- `python3 -m serial.tools.list_ports -v` => `no ports found`
- `/dev/bus/usb` is visible, but no `/dev/ttyACM*`/`/dev/ttyUSB*` node exists in container

## Conclusion

Software dependencies are ready. Current blockers are runtime resource mapping:

1. STM32 CDC serial device is not passed into Docker (`/dev/ttyACM0` missing)
2. ONNX model and normalizer stats files are not mounted/copied into container

## Suggested Relaunch Options

### Option A (minimum)

```bash
docker run --rm -it \
  --device=/dev/ttyACM0:/dev/ttyACM0 \
  -v /home/user/IsaacLab/sim2real_test/config:/home/user/IsaacLab/sim2real_test/config:ro \
  -v /path/to/weixue-dog-nano-code:/workspace/weixue-dog-nano-code \
  <your-image>
```

### Option B (debug convenience)

```bash
docker run --rm -it --privileged \
  -v /dev:/dev \
  -v /home/user/IsaacLab/sim2real_test/config:/home/user/IsaacLab/sim2real_test/config:ro \
  -v /path/to/weixue-dog-nano-code:/workspace/weixue-dog-nano-code \
  <your-image>
```

## Re-test Commands After Relaunch

```bash
cd /workspace/weixue-dog-nano-code/sim2real_jetson_tests
python3 01_env_check.py --port /dev/ttyACM0
python3 02_stm32_state_probe.py --port /dev/ttyACM0 --duration 5
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50
python3 03_single_servo_safe_test.py --port /dev/ttyACM0 --id 1 --delta 50 --execute
python3 04_onnx_latency_smoke_test.py \
  --model /home/user/IsaacLab/sim2real_test/config/wavego_policy.onnx \
  --stats /home/user/IsaacLab/sim2real_test/config/normalizer_stats.npz \
  --iters 500
```
