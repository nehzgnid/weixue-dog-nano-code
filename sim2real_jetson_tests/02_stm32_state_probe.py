#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_io import RobotIO  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32 状态链路探测（IMU+舵机反馈）")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--print-every", type=float, default=0.2)
    args = parser.parse_args()

    io = RobotIO(port=args.port, baud_rate=args.baud)
    if io.ser is None:
        print("[FAIL] 串口未连接成功")
        return 1

    print(f"[INFO] 已连接: {args.port} @ {args.baud}")
    print("[INFO] 开始读取状态...（仅观测，不发运动指令）")

    t0 = time.perf_counter()
    last_print = 0.0
    imu_updates = 0
    servo_updates = 0

    try:
        while time.perf_counter() - t0 < args.duration:
            now = time.perf_counter() - t0
            imu = io.get_imu()
            with io.lock:
                states = dict(io.servo_states)

            if any(abs(imu[k]) > 1e-6 for k in ("roll", "pitch", "yaw")):
                imu_updates += 1
            if len(states) > 0:
                servo_updates += 1

            if now - last_print >= args.print_every:
                last_print = now
                sid1 = states.get(1, {})
                p1 = sid1.get("pos", None)
                print(
                    f"t={now:5.2f}s | imu(r,p,y)=({imu['roll']:+6.2f},{imu['pitch']:+6.2f},{imu['yaw']:+6.2f})"
                    f" | servo_count={len(states):2d} | id1_pos={p1}"
                )

            time.sleep(0.01)
    finally:
        io.close()

    ok = imu_updates > 0 or servo_updates > 0
    if ok:
        print("[PASS] 状态链路有数据更新，可进入步骤 03/04")
        return 0

    print("[FAIL] 未观察到状态更新，请检查固件协议/串口接线")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
