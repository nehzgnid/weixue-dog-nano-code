#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import numpy as np
import onnxruntime as ort


def main() -> int:
    parser = argparse.ArgumentParser(description="ONNX 推理与全链路轻量测试")
    parser.add_argument("--model", default="/home/user/IsaacLab/sim2real_test/config/wavego_policy.onnx")
    parser.add_argument("--stats", default="/home/user/IsaacLab/sim2real_test/config/normalizer_stats.npz")
    parser.add_argument("--iters", type=int, default=500)
    args = parser.parse_args()

    stats = np.load(args.stats)
    mean = stats["mean"].astype(np.float32)
    std = stats["std"].astype(np.float32)

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    dummy = np.zeros((1, 48), dtype=np.float32)
    for _ in range(30):
        sess.run(None, {in_name: dummy})

    infer_times = []
    full_times = []
    last_out = None

    for _ in range(args.iters):
        obs = np.random.randn(48).astype(np.float32)

        t0 = time.perf_counter()
        out = sess.run(None, {in_name: obs.reshape(1, -1)})[0]
        infer_times.append((time.perf_counter() - t0) * 1000.0)
        last_out = out

        t1 = time.perf_counter()
        obs_norm = np.clip((obs - mean) / (std + 1e-8), -5.0, 5.0)
        _ = sess.run(None, {in_name: obs_norm.reshape(1, -1)})[0]
        full_times.append((time.perf_counter() - t1) * 1000.0)

    infer = np.array(infer_times)
    full = np.array(full_times)

    print("=== ONNX 延迟测试 ===")
    print(f"provider={sess.get_providers()[0]}")
    print(f"infer mean={infer.mean():.4f}ms p95={np.percentile(infer,95):.4f}ms p99={np.percentile(infer,99):.4f}ms")
    print(f"full  mean={full.mean():.4f}ms p95={np.percentile(full,95):.4f}ms p99={np.percentile(full,99):.4f}ms")
    print(f"last output shape={None if last_out is None else last_out.shape}")

    if infer.mean() < 5.0:
        print("[PASS] 推理时延满足 50Hz 控制预算")
        return 0

    print("[WARN] 推理时延偏高，请检查环境/负载")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
