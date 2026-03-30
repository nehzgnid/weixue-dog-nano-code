"""
端到端延迟测试 — 在 Jetson Nano 上测量推理与通信延迟。

用法:
    python test_latency.py --model config/wavego_policy.onnx --iterations 1000
"""

from __future__ import annotations

import argparse
import time

import numpy as np

NUM_OBS = 48


def test_onnx_latency(model_path: str, iterations: int = 1000, use_gpu: bool = True):
    """测量 ONNX 推理延迟。"""
    import onnxruntime as ort

    providers = (
        ["CUDAExecutionProvider", "CPUExecutionProvider"]
        if use_gpu
        else ["CPUExecutionProvider"]
    )
    sess = ort.InferenceSession(model_path, providers=providers)
    inp_name = sess.get_inputs()[0].name
    actual_provider = sess.get_providers()[0]

    print(f"Model: {model_path}")
    print(f"Provider: {actual_provider}")
    print(f"Iterations: {iterations}")

    dummy = np.zeros((1, NUM_OBS), dtype=np.float32)

    # 预热 (排除首次加载开销)
    for _ in range(20):
        sess.run(None, {inp_name: dummy})

    # 测速
    times = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        sess.run(None, {inp_name: dummy})
        times.append(time.perf_counter() - t0)

    times_ms = np.array(times) * 1000
    print(f"\n--- ONNX Inference Latency ---")
    print(f"Mean:   {times_ms.mean():.3f} ms")
    print(f"Median: {np.median(times_ms):.3f} ms")
    print(f"P95:    {np.percentile(times_ms, 95):.3f} ms")
    print(f"P99:    {np.percentile(times_ms, 99):.3f} ms")
    print(f"Max:    {times_ms.max():.3f} ms")
    print(f"Min:    {times_ms.min():.3f} ms")
    print(f"Std:    {times_ms.std():.3f} ms")

    # 判定
    if times_ms.mean() < 5.0:
        print("\n✅ PASS: 平均推理 < 5ms，满足 50Hz 控制要求")
    else:
        print("\n⚠️ WARN: 平均推理 >= 5ms，可能影响控制频率")


def test_normalize_latency(stats_path: str, iterations: int = 10000):
    """测量 normalize 计算延迟。"""
    data = np.load(stats_path)
    mean = data["mean"].astype(np.float32)
    std = data["std"].astype(np.float32)

    obs = np.random.randn(NUM_OBS).astype(np.float32)

    times = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        _ = np.clip((obs - mean) / (std + 1e-8), -5.0, 5.0)
        times.append(time.perf_counter() - t0)

    times_us = np.array(times) * 1e6
    print(f"\n--- Normalize Latency ---")
    print(f"Mean:   {times_us.mean():.1f} µs")
    print(f"Max:    {times_us.max():.1f} µs")


def test_observation_build_latency(iterations: int = 10000):
    """测量观测拼接延迟。"""
    lin_vel = np.zeros(3, dtype=np.float32)
    ang_vel = np.zeros(3, dtype=np.float32)
    grav = np.array([0, 0, -1], dtype=np.float32)
    cmd = np.zeros(3, dtype=np.float32)
    jpos = np.zeros(12, dtype=np.float32)
    jvel = np.zeros(12, dtype=np.float32)
    act = np.zeros(12, dtype=np.float32)

    times = []
    for _ in range(iterations):
        t0 = time.perf_counter()
        _ = np.concatenate([lin_vel, ang_vel, grav, cmd, jpos, jvel, act])
        times.append(time.perf_counter() - t0)

    times_us = np.array(times) * 1e6
    print(f"\n--- Observation Build Latency ---")
    print(f"Mean:   {times_us.mean():.1f} µs")
    print(f"Max:    {times_us.max():.1f} µs")


def test_full_pipeline_latency(
    model_path: str, stats_path: str, iterations: int = 1000, use_gpu: bool = True
):
    """测量完整推理管线延迟（不含硬件 IO）。"""
    import onnxruntime as ort

    providers = (
        ["CUDAExecutionProvider", "CPUExecutionProvider"]
        if use_gpu
        else ["CPUExecutionProvider"]
    )
    sess = ort.InferenceSession(model_path, providers=providers)
    inp_name = sess.get_inputs()[0].name

    data = np.load(stats_path)
    mean = data["mean"].astype(np.float32)
    std = data["std"].astype(np.float32)

    # 预热
    dummy = np.zeros((1, NUM_OBS), dtype=np.float32)
    for _ in range(20):
        sess.run(None, {inp_name: dummy})

    times = []
    for _ in range(iterations):
        obs_raw = np.random.randn(NUM_OBS).astype(np.float32)
        t0 = time.perf_counter()

        # 1. normalize
        obs_norm = np.clip((obs_raw - mean) / (std + 1e-8), -5.0, 5.0)
        # 2. inference
        action = sess.run(None, {inp_name: obs_norm.reshape(1, -1)})[0].squeeze()
        # 3. action scale
        _ = obs_raw[:12] + action * 0.25

        times.append(time.perf_counter() - t0)

    times_ms = np.array(times) * 1000
    print(f"\n--- Full Pipeline Latency (normalize + inference + action_scale) ---")
    print(f"Mean:   {times_ms.mean():.3f} ms")
    print(f"P95:    {np.percentile(times_ms, 95):.3f} ms")
    print(f"P99:    {np.percentile(times_ms, 99):.3f} ms")
    print(f"Max:    {times_ms.max():.3f} ms")

    budget_ms = 20.0  # 50 Hz
    print(f"\nRemaining for sensor IO: {budget_ms - times_ms.mean():.1f} ms (of {budget_ms:.0f} ms budget)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Latency benchmark on Jetson Nano")
    parser.add_argument("--model", type=str, default="config/wavego_policy.onnx")
    parser.add_argument("--stats", type=str, default="config/normalizer_stats.npz")
    parser.add_argument("--iterations", type=int, default=1000)
    parser.add_argument("--no-gpu", action="store_true")
    parser.add_argument("--full", action="store_true", help="Run full pipeline test")
    args = parser.parse_args()

    test_onnx_latency(args.model, args.iterations, use_gpu=not args.no_gpu)
    test_normalize_latency(args.stats)
    test_observation_build_latency()

    if args.full:
        test_full_pipeline_latency(args.model, args.stats, args.iterations, use_gpu=not args.no_gpu)
