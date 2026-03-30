#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from common.io.robot_io import RobotIO  # noqa: E402


def clamp(value: int, low: int = 0, high: int = 4095) -> int:
    return max(low, min(high, value))


def main() -> int:
    parser = argparse.ArgumentParser(description="单舵机安全微动测试（默认 dry-run）")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--id", type=int, default=1, help="舵机 ID")
    parser.add_argument("--delta", type=int, default=50, help="PWM 偏移，建议 20~80")
    parser.add_argument("--hold", type=float, default=0.8, help="保持时间（秒）")
    parser.add_argument("--execute", action="store_true", help="确认实际执行动作")
    args = parser.parse_args()

    if not args.execute:
        print("[SAFE] 当前为 dry-run，仅打印将执行的指令。加 --execute 才会真实发送。")

    io = RobotIO(port=args.port, baud_rate=args.baud)
    if io.ser is None:
        print("[FAIL] 串口未连接成功")
        return 1

    time.sleep(0.2)
    with io.lock:
        st = dict(io.servo_states)

    current = st.get(args.id, {}).get("pos", 2048)
    target = clamp(int(current) + int(args.delta))
    back = clamp(int(current))

    print(f"[PLAN] id={args.id}, current={current}, target={target}, back={back}, hold={args.hold}s")

    if args.execute:
        try:
            io.send_torque(True)
            time.sleep(0.1)
            io.send_servos([(args.id, target, 300, 0, 0)])
            time.sleep(args.hold)
            io.send_servos([(args.id, back, 300, 0, 0)])
            time.sleep(0.2)
            print("[PASS] 单舵机动作执行完成")
        finally:
            io.send_torque(False)
            io.close()
    else:
        io.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
