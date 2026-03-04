#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import os
import platform
import sys
from pathlib import Path


def check_import(module_name: str) -> tuple[bool, str]:
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, "__version__", "unknown")
        return True, str(version)
    except Exception as exc:
        return False, str(exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="Jetson Nano sim2real 环境快速检查")
    parser.add_argument("--port", default="/dev/ttyACM0", help="STM32 CDC 设备")
    parser.add_argument("--python-minor", type=int, default=8, help="最低 Python 小版本")
    args = parser.parse_args()

    print("=== 01 环境检查 ===")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Platform: {platform.platform()}")
    print(f"Machine: {platform.machine()}")

    ok = True

    if sys.version_info.major != 3 or sys.version_info.minor < args.python_minor:
        print(f"[FAIL] Python 版本过低，需要 >= 3.{args.python_minor}")
        ok = False
    else:
        print(f"[PASS] Python 版本满足 >= 3.{args.python_minor}")

    for module in ["numpy", "serial", "yaml", "onnxruntime"]:
        imported, detail = check_import(module)
        if imported:
            print(f"[PASS] import {module} ({detail})")
        else:
            if module == "onnxruntime":
                print(f"[WARN] import {module} 失败: {detail}")
                print("       若当前仅做串口链路测试可暂不安装；推理测试必须安装")
            else:
                print(f"[FAIL] import {module} 失败: {detail}")
                ok = False

    port = Path(args.port)
    if port.exists():
        print(f"[PASS] 串口设备存在: {port}")
    else:
        print(f"[FAIL] 串口设备不存在: {port}")
        ok = False

    if port.exists():
        can_rw = os.access(port, os.R_OK | os.W_OK)
        if can_rw:
            print(f"[PASS] 串口读写权限正常: {port}")
        else:
            print(f"[FAIL] 串口权限不足: {port}")
            ok = False

    print("===================")
    if ok:
        print("环境检查通过，可进入步骤 02")
        return 0
    print("环境检查未通过，请先修复 FAIL 项")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
