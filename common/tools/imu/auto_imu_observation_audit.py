#!/usr/bin/env python3
"""
自动审计 IMU -> 训练观测转化逻辑。

目标
----
1) 采集 RAW IMU 数据（acc/gyro/rpy）
2) 通过动作引导（俯仰/侧倾/平转）自动估计轴映射
3) 穷举候选转换，评分并输出最佳映射
4) 对比当前仓库默认配置，给出 PASS/FAIL 结论

重要约束
--------
- 用户训练坐标: Y 轴前向，右手系
- 机械安装: IMU 的 Y+ 指向机器狗前方

脚本默认会启用 "Y 轴前向约束"（obs_y 必须来自 raw_y），
你也可用 --no-y-forward-constraint 关闭该约束做全量搜索。

使用示例
--------
python auto_imu_observation_audit.py
python auto_imu_observation_audit.py /dev/ttyACM0
python auto_imu_observation_audit.py --no-y-forward-constraint
python auto_imu_observation_audit.py --static-sec 5 --motion-sec 8 --save-report
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    import yaml
except Exception:
    yaml = None

# 复用同目录串口读取实现（协议与现有工具一致）
try:
    from .test_imu_readout import IMUReader
except Exception:
    from test_imu_readout import IMUReader


AXIS_NAME = ["X", "Y", "Z"]
RPY_NAME = ["Roll", "Pitch"]


@dataclass(frozen=True)
class EulerMap:
    roll_src: int   # 0=raw_roll, 1=raw_pitch
    roll_sign: float
    pitch_src: int  # 0=raw_roll, 1=raw_pitch
    pitch_sign: float
    label: str


@dataclass
class CandidateResult:
    perm: tuple[int, int, int]
    signs: tuple[float, float, float]
    euler_map: EulerMap
    handedness: int
    axis_score: float
    corr_score: float
    gravity_err: float
    total_score: float


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="自动审计 IMU 观测坐标转换")
    p.add_argument("port", nargs="?", default=None, help="串口设备，例如 /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200, help="串口波特率")
    p.add_argument("--static-sec", type=float, default=4.0, help="静止采样秒数")
    p.add_argument("--motion-sec", type=float, default=6.0, help="每个动作采样秒数")
    p.add_argument(
        "--min-motion-rms",
        type=float,
        default=0.8,
        help="动作段最小陀螺 RMS 门槛(deg/s)，低于该值判定动作为无效",
    )
    p.add_argument(
        "--min-dominance-ratio",
        type=float,
        default=1.25,
        help="每段动作主导轴与次主导轴的最小比值，低于该值判定动作不够纯",
    )
    p.add_argument(
        "--no-y-forward-constraint",
        action="store_true",
        help="关闭 Y 前向约束（允许 obs_y 不来自 raw_y）",
    )
    p.add_argument(
        "--save-report",
        action="store_true",
        help="保存 Markdown 报告到 traditional_control_tests/",
    )
    p.add_argument(
        "--config",
        default=None,
        help="指定 wavego_deploy_config.yaml 路径；默认自动定位仓库配置",
    )
    p.add_argument(
        "--allow-left-handed",
        action="store_true",
        help="允许左手系候选参与搜索（默认仅保留右手系）",
    )
    return p.parse_args(argv)


def _perm_parity(perm: tuple[int, int, int]) -> int:
    inv = 0
    for i in range(3):
        for j in range(i + 1, 3):
            if perm[i] > perm[j]:
                inv += 1
    return 1 if (inv % 2 == 0) else -1


def _transform_handedness(perm: tuple[int, int, int], signs: tuple[float, float, float]) -> int:
    sign_prod = 1
    for v in signs:
        sign_prod *= 1 if v > 0 else -1
    return _perm_parity(perm) * sign_prod


def load_repo_observation_config(config_path: str | None):
    """从仓库配置加载当前 remap/sign，失败时回退到现有默认值。"""
    fallback_perm = (1, 0, 2)
    fallback_signs = (1.0, -1.0, 1.0)

    if config_path is None:
        cfg_path = (
            Path(__file__).resolve().parent.parent
            / "sim2real_jetson_tests"
            / "strategy_deploy_bundle"
            / "config"
            / "wavego_deploy_config.yaml"
        )
    else:
        cfg_path = Path(config_path)

    if yaml is None:
        return fallback_perm, fallback_signs, None, "PyYAML 不可用，使用回退默认值"

    if not cfg_path.exists():
        return fallback_perm, fallback_signs, None, f"配置文件不存在: {cfg_path}，使用回退默认值"

    try:
        with cfg_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        obs = data.get("observation", {})
        perm_raw = obs.get("imu_axis_remap", list(fallback_perm))
        signs_raw = obs.get("imu_axis_sign", list(fallback_signs))
        if len(perm_raw) != 3 or len(signs_raw) != 3:
            raise ValueError("imu_axis_remap/imu_axis_sign 维度必须为 3")
        perm = tuple(int(x) for x in perm_raw)
        signs = tuple(float(x) for x in signs_raw)
        return perm, signs, cfg_path, None
    except Exception as e:
        return fallback_perm, fallback_signs, None, f"读取配置失败({e})，使用回退默认值"


def wait_enter(message: str):
    print("\n" + "=" * 72)
    print(message)
    print("按回车开始采样...")
    input()


def collect_segment(reader: IMUReader, name: str, seconds: float) -> np.ndarray:
    """
    采集段数据，返回 shape=(N, 10)
    列定义: t, acc_x,acc_y,acc_z, gyro_x,gyro_y,gyro_z, roll,pitch,yaw
    """
    t_end = time.perf_counter() + seconds
    rows = []
    last_tick = None

    while True:
        now = time.perf_counter()
        if now >= t_end:
            break

        if now - reader.last_request_time >= 0.05:
            reader.request_state()

        s = reader.get_snapshot()
        if s["fresh"]:
            rows.append(
                [
                    now,
                    s["accel_raw"][0],
                    s["accel_raw"][1],
                    s["accel_raw"][2],
                    s["gyro_raw"][0],
                    s["gyro_raw"][1],
                    s["gyro_raw"][2],
                    s["roll"],
                    s["pitch"],
                    s["yaw_raw"],
                ]
            )

        remain = max(0.0, t_end - now)
        tick = int(math.ceil(remain))
        if tick != last_tick:
            sys.stdout.write(f"\r{name} 采样中: 剩余 {tick:2d}s")
            sys.stdout.flush()
            last_tick = tick

        time.sleep(0.01)

    sys.stdout.write("\r" + " " * 40 + "\r")
    sys.stdout.flush()

    if not rows:
        return np.zeros((0, 10), dtype=np.float64)
    return np.array(rows, dtype=np.float64)


def split_cols(seg: np.ndarray):
    t = seg[:, 0]
    acc = seg[:, 1:4]
    gyro = seg[:, 4:7]
    roll = seg[:, 7]
    pitch = seg[:, 8]
    yaw = seg[:, 9]
    return t, acc, gyro, roll, pitch, yaw


def projected_gravity_deg(obs_roll_deg: np.ndarray, obs_pitch_deg: np.ndarray) -> np.ndarray:
    """与 obs_builder.py 同公式，输入角度单位为 deg，输出单位向量(g)。"""
    rr = np.deg2rad(obs_roll_deg)
    rp = np.deg2rad(obs_pitch_deg)
    cr = np.cos(rr)
    sr = np.sin(rr)
    cp = np.cos(rp)
    sp = np.sin(rp)
    gx = -sp
    gy = cp * sr
    gz = -cp * cr
    return np.stack([gx, gy, gz], axis=1)


def remap_vec(v_raw: np.ndarray, perm: tuple[int, int, int], signs: tuple[float, float, float]) -> np.ndarray:
    s = np.array(signs, dtype=np.float64)
    return v_raw[:, list(perm)] * s[None, :]


def map_euler(raw_roll: np.ndarray, raw_pitch: np.ndarray, e: EulerMap) -> tuple[np.ndarray, np.ndarray]:
    src = np.stack([raw_roll, raw_pitch], axis=1)
    obs_roll = e.roll_sign * src[:, e.roll_src]
    obs_pitch = e.pitch_sign * src[:, e.pitch_src]
    return obs_roll, obs_pitch


def safe_corr(x: np.ndarray, y: np.ndarray) -> float:
    if x.size < 8 or y.size < 8:
        return 0.0
    x0 = x - np.mean(x)
    y0 = y - np.mean(y)
    sx = float(np.std(x0))
    sy = float(np.std(y0))
    if sx < 1e-9 or sy < 1e-9:
        return 0.0
    return float(np.mean((x0 / sx) * (y0 / sy)))


def axis_energy_score(
    perm: tuple[int, int, int],
    rms_pitch: np.ndarray,
    rms_roll: np.ndarray,
    rms_yaw: np.ndarray,
) -> float:
    """
    评分目标：
    - 俯仰动作 -> obs X 轴角速度应最强
    - 侧倾动作 -> obs Y 轴角速度应最强
    - 平转动作 -> obs Z 轴角速度应最强
    """
    seg_rms = [rms_pitch, rms_roll, rms_yaw]
    expected_obs_axis = [0, 1, 2]  # X, Y, Z

    score = 0.0
    for seg_idx, obs_axis in enumerate(expected_obs_axis):
        chosen_raw = perm[obs_axis]
        rms = seg_rms[seg_idx]
        sel = float(rms[chosen_raw])
        total = float(np.sum(rms)) + 1e-9
        others = np.delete(rms, chosen_raw)
        leak = float(np.mean(others))
        ratio = sel / total
        score += 1.3 * ratio + 0.35 * (sel - leak)
    return score


def iter_euler_maps() -> Iterable[EulerMap]:
    maps = []
    for swap in [False, True]:
        for sr in [-1.0, 1.0]:
            for sp in [-1.0, 1.0]:
                if not swap:
                    roll_src, pitch_src = 0, 1
                else:
                    roll_src, pitch_src = 1, 0
                label = (
                    f"obs_roll={'-' if sr < 0 else '+'}raw_{RPY_NAME[roll_src].lower()}, "
                    f"obs_pitch={'-' if sp < 0 else '+'}raw_{RPY_NAME[pitch_src].lower()}"
                )
                maps.append(EulerMap(roll_src, sr, pitch_src, sp, label))
    return maps


def evaluate_candidate(
    perm: tuple[int, int, int],
    signs: tuple[float, float, float],
    euler_map: EulerMap,
    handedness: int,
    seg_static: np.ndarray,
    seg_pitch: np.ndarray,
    seg_roll: np.ndarray,
    seg_yaw: np.ndarray,
    gyro_bias: np.ndarray,
    axis_score: float,
) -> CandidateResult:
    # --- 重力一致性（静止段） ---
    t_s, acc_s, _gyro_s, raw_roll_s, raw_pitch_s, _yaw_s = split_cols(seg_static)
    obs_acc_s = remap_vec(acc_s, perm, signs)
    obs_roll_s, obs_pitch_s = map_euler(raw_roll_s, raw_pitch_s, euler_map)
    g_proj_s = projected_gravity_deg(obs_roll_s, obs_pitch_s)
    gravity_err = float(np.median(np.linalg.norm(obs_acc_s + g_proj_s, axis=1)))

    # --- 角速度 vs 欧拉角导数一致性（动态段） ---
    # pitch 动作应主要体现 obs_pitch 对应的 x 轴角速度
    # roll 动作应主要体现 obs_roll  对应的 y 轴角速度
    def corr_for_segment(seg: np.ndarray, obs_axis: int, use_obs_pitch: bool) -> float:
        t, _acc, gyro, raw_roll, raw_pitch, _yaw = split_cols(seg)
        obs_gyro = remap_vec(gyro - gyro_bias[None, :], perm, signs)
        obs_roll, obs_pitch = map_euler(raw_roll, raw_pitch, euler_map)
        angle = obs_pitch if use_obs_pitch else obs_roll
        d_angle = np.gradient(angle, t)
        return safe_corr(obs_gyro[:, obs_axis], d_angle)

    corr_pitch = corr_for_segment(seg_pitch, obs_axis=0, use_obs_pitch=True)
    corr_roll = corr_for_segment(seg_roll, obs_axis=1, use_obs_pitch=False)
    corr_score = corr_pitch + corr_roll

    # 总分：越大越好
    total_score = 2.0 * axis_score + 1.4 * corr_score - 3.2 * gravity_err

    return CandidateResult(
        perm=perm,
        signs=signs,
        euler_map=euler_map,
        handedness=handedness,
        axis_score=axis_score,
        corr_score=corr_score,
        gravity_err=gravity_err,
        total_score=total_score,
    )


def fmt_vector_map(perm: tuple[int, int, int], signs: tuple[float, float, float]) -> str:
    terms = []
    for obs_i, raw_i in enumerate(perm):
        s = "+" if signs[obs_i] > 0 else "-"
        terms.append(f"OBS_{AXIS_NAME[obs_i]} = {s}RAW_{AXIS_NAME[raw_i]}")
    return " | ".join(terms)


def candidate_key(c: CandidateResult):
    return c.total_score


def run_audit(args: argparse.Namespace) -> int:
    reader = IMUReader(port=args.port, baud=args.baud)

    print("正在连接串口...")
    if not reader.connect():
        if not reader.reconnect():
            print("[FAIL] 无法连接串口设备")
            return 2

    print(f"已连接: {reader.port}")
    print("\n自动审计流程将采集 4 个阶段: 静止 / 俯仰 / 侧倾 / 平转")
    print("请尽量在每个动作阶段只做该动作，幅度 10°~30°，速度中等。")

    try:
        wait_enter(f"步骤 1/4: 保持机器人静止 {args.static_sec:.1f}s")
        seg_static = collect_segment(reader, "静止", args.static_sec)

        wait_enter(f"步骤 2/4: 仅做机头上下(俯仰) {args.motion_sec:.1f}s")
        seg_pitch = collect_segment(reader, "俯仰", args.motion_sec)

        wait_enter(f"步骤 3/4: 仅做左右侧倾(滚转) {args.motion_sec:.1f}s")
        seg_roll = collect_segment(reader, "侧倾", args.motion_sec)

        wait_enter(f"步骤 4/4: 保持基本水平，做原地左右旋转(偏航) {args.motion_sec:.1f}s")
        seg_yaw = collect_segment(reader, "平转", args.motion_sec)

    finally:
        reader.close()

    min_required = 30
    segs = {
        "静止": seg_static,
        "俯仰": seg_pitch,
        "侧倾": seg_roll,
        "平转": seg_yaw,
    }
    for k, v in segs.items():
        if v.shape[0] < min_required:
            print(f"[FAIL] {k} 段有效样本太少: {v.shape[0]} < {min_required}")
            print("请重试，并确认串口数据稳定（状态为 fresh）。")
            return 3

    # 基础统计
    _, _, gyro_static, *_ = split_cols(seg_static)
    gyro_bias = np.mean(gyro_static, axis=0)

    _, _, gyro_pitch, *_ = split_cols(seg_pitch)
    _, _, gyro_roll, *_ = split_cols(seg_roll)
    _, _, gyro_yaw, *_ = split_cols(seg_yaw)

    rms_pitch = np.sqrt(np.mean((gyro_pitch - gyro_bias[None, :]) ** 2, axis=0))
    rms_roll = np.sqrt(np.mean((gyro_roll - gyro_bias[None, :]) ** 2, axis=0))
    rms_yaw = np.sqrt(np.mean((gyro_yaw - gyro_bias[None, :]) ** 2, axis=0))

    print("\n=== 段内 RAW 陀螺轴能量 (deg/s RMS, 去静态偏置后) ===")
    print(f"俯仰段: X={rms_pitch[0]:.3f}, Y={rms_pitch[1]:.3f}, Z={rms_pitch[2]:.3f}")
    print(f"侧倾段: X={rms_roll[0]:.3f}, Y={rms_roll[1]:.3f}, Z={rms_roll[2]:.3f}")
    print(f"平转段: X={rms_yaw[0]:.3f}, Y={rms_yaw[1]:.3f}, Z={rms_yaw[2]:.3f}")

    # 动作质量门槛：若动作激励不足，自动判定本次结果无效，避免误判。
    motion_peaks = {
        "俯仰": float(np.max(rms_pitch)),
        "侧倾": float(np.max(rms_roll)),
        "平转": float(np.max(rms_yaw)),
    }
    weak_segments = [k for k, v in motion_peaks.items() if v < args.min_motion_rms]
    if weak_segments:
        print("\n[FAIL] 动作激励不足，无法做严格审计。")
        print(f"最小门槛: {args.min_motion_rms:.2f} deg/s")
        for k in ["俯仰", "侧倾", "平转"]:
            print(f"- {k} 峰值 RMS: {motion_peaks[k]:.3f} deg/s")
        print(f"请重跑并增大动作幅度/速度，问题段: {', '.join(weak_segments)}")
        return 5

    # 动作可分辨性门槛：避免三个动作段都激励到同一原始轴导致误判。
    def _dominance_info(rms: np.ndarray):
        idx = np.argsort(rms)[::-1]
        major = int(idx[0])
        ratio = float(rms[idx[0]] / (rms[idx[1]] + 1e-9))
        return major, ratio

    dom_pitch, ratio_pitch = _dominance_info(rms_pitch)
    dom_roll, ratio_roll = _dominance_info(rms_roll)
    dom_yaw, ratio_yaw = _dominance_info(rms_yaw)

    low_ratio = []
    if ratio_pitch < args.min_dominance_ratio:
        low_ratio.append(f"俯仰({ratio_pitch:.2f})")
    if ratio_roll < args.min_dominance_ratio:
        low_ratio.append(f"侧倾({ratio_roll:.2f})")
    if ratio_yaw < args.min_dominance_ratio:
        low_ratio.append(f"平转({ratio_yaw:.2f})")
    if low_ratio:
        print("\n[FAIL] 动作纯度不足，无法严格区分各旋转轴。")
        print(f"最小主导轴比值门槛: {args.min_dominance_ratio:.2f}")
        print(f"当前比值: 俯仰={ratio_pitch:.2f}, 侧倾={ratio_roll:.2f}, 平转={ratio_yaw:.2f}")
        print(f"请重跑并确保每段只做单一动作，问题段: {', '.join(low_ratio)}")
        return 6

    dom_set = {dom_pitch, dom_roll, dom_yaw}
    if len(dom_set) < 3:
        print("\n[FAIL] 三个动作段的主导 RAW 轴不互异，动作分段可能执行错误。")
        print(f"主导轴: 俯仰={AXIS_NAME[dom_pitch]}, 侧倾={AXIS_NAME[dom_roll]}, 平转={AXIS_NAME[dom_yaw]}")
        print("请重跑并按引导分别做俯仰/侧倾/平转。")
        return 7

    # 穷举候选
    perms_all = list(itertools.permutations([0, 1, 2], 3))
    if args.no_y_forward_constraint:
        perms = perms_all
    else:
        # 用户先验：IMU Y+ 指向机头前方，训练坐标 Y 前向
        perms = [p for p in perms_all if p[1] == 1]

    signs_all = list(itertools.product([-1.0, 1.0], repeat=3))
    euler_maps = list(iter_euler_maps())

    results: list[CandidateResult] = []
    for perm in perms:
        axis_score = axis_energy_score(perm, rms_pitch, rms_roll, rms_yaw)
        for signs in signs_all:
            handedness = _transform_handedness(perm, signs)
            if (not args.allow_left_handed) and handedness < 0:
                continue
            for e_map in euler_maps:
                c = evaluate_candidate(
                    perm,
                    signs,
                    e_map,
                    handedness,
                    seg_static,
                    seg_pitch,
                    seg_roll,
                    seg_yaw,
                    gyro_bias,
                    axis_score,
                )
                results.append(c)

    if not results:
        print("[FAIL] 没有可用候选，请检查参数设置")
        return 4

    results.sort(key=candidate_key, reverse=True)
    best = results[0]

    # 当前仓库配置（优先读取 wavego_deploy_config.yaml）
    current_perm, current_signs, loaded_cfg_path, cfg_warn = load_repo_observation_config(args.config)
    if cfg_warn:
        print(f"[WARN] {cfg_warn}")

    current_euler = EulerMap(roll_src=1, roll_sign=-1.0, pitch_src=0, pitch_sign=1.0,
                             label="obs_roll=-raw_pitch, obs_pitch=+raw_roll")
    current_axis_score = axis_energy_score(current_perm, rms_pitch, rms_roll, rms_yaw)
    current_handedness = _transform_handedness(current_perm, current_signs)
    current = evaluate_candidate(
        current_perm,
        current_signs,
        current_euler,
        current_handedness,
        seg_static,
        seg_pitch,
        seg_roll,
        seg_yaw,
        gyro_bias,
        current_axis_score,
    )

    # 当前候选在本次搜索集合中的名次（若受约束被排除则记 None）
    rank_current = None
    for i, c in enumerate(results, start=1):
        if c.perm == current.perm and c.signs == current.signs and c.euler_map == current.euler_map:
            rank_current = i
            break

    print("\n=== 自动审计结果 ===")
    print(f"搜索约束: {'Y 前向约束已关闭' if args.no_y_forward_constraint else 'Y 前向约束已启用(OBS_Y 来自 RAW_Y)'}")
    print(f"手性约束: {'允许左手系' if args.allow_left_handed else '仅右手系'}")
    print("\n[最佳候选]")
    print(f"imu_axis_remap = {list(best.perm)}")
    print(f"imu_axis_sign  = {[float(x) for x in best.signs]}")
    print(f"euler_map      = {best.euler_map.label}")
    print(f"handedness     = {'右手系' if best.handedness > 0 else '左手系'}")
    print(f"vector_map     = {fmt_vector_map(best.perm, best.signs)}")
    print(
        f"score(total/axis/corr/gravity_err)="
        f"{best.total_score:.3f}/{best.axis_score:.3f}/{best.corr_score:.3f}/{best.gravity_err:.4f}"
    )

    print("\n[当前仓库配置候选]")
    if loaded_cfg_path is not None:
        print(f"config_source  = {loaded_cfg_path}")
    print(f"imu_axis_remap = {list(current.perm)}")
    print(f"imu_axis_sign  = {[float(x) for x in current.signs]}")
    print(f"euler_map      = {current.euler_map.label}")
    print(f"handedness     = {'右手系' if current.handedness > 0 else '左手系'}")
    print(f"vector_map     = {fmt_vector_map(current.perm, current.signs)}")
    print(
        f"score(total/axis/corr/gravity_err)="
        f"{current.total_score:.3f}/{current.axis_score:.3f}/{current.corr_score:.3f}/{current.gravity_err:.4f}"
    )
    if rank_current is None:
        print("rank_in_search = (当前配置不满足本次约束，未参与排名)")
    else:
        print(f"rank_in_search = {rank_current}/{len(results)}")

    # 给出结论
    score_gap = best.total_score - current.total_score
    same_transform = (
        best.perm == current.perm
        and best.signs == current.signs
        and best.euler_map == current.euler_map
    )

    print("\n[结论]")
    if same_transform:
        print("PASS: 最佳候选与当前仓库配置一致。")
    else:
        print("FAIL: 当前仓库配置不是本次数据下的最佳候选。")
        print(f"      最佳候选相对当前配置分数提升: {score_gap:+.3f}")

    if not args.no_y_forward_constraint and best.perm[1] != 1:
        print("WARN: 最佳候选不满足 Y 前向约束，请检查安装描述或动作采样质量。")

    if args.save_report:
        save_report(
            best,
            current,
            rank_current,
            results,
            args,
            rms_pitch,
            rms_roll,
            rms_yaw,
            loaded_cfg_path,
            cfg_warn,
        )

    return 0


def save_report(
    best: CandidateResult,
    current: CandidateResult,
    rank_current: int | None,
    results: list[CandidateResult],
    args: argparse.Namespace,
    rms_pitch: np.ndarray,
    rms_roll: np.ndarray,
    rms_yaw: np.ndarray,
    loaded_cfg_path: Path | None,
    cfg_warn: str | None,
):
    ts = time.strftime("%Y%m%d_%H%M%S")
    out = Path(__file__).resolve().parent / f"IMU_OBS_AUDIT_REPORT_{ts}.md"

    lines = []
    lines.append("# IMU 观测坐标自动审计报告")
    lines.append("")
    lines.append(f"- 时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"- 串口: {args.port or 'auto'}")
    lines.append(f"- Y 前向约束: {'关闭' if args.no_y_forward_constraint else '开启'}")
    lines.append(f"- 手性约束: {'允许左手系' if args.allow_left_handed else '仅右手系'}")
    if loaded_cfg_path is not None:
        lines.append(f"- 当前配置来源: {loaded_cfg_path}")
    if cfg_warn:
        lines.append(f"- 配置读取警告: {cfg_warn}")
    lines.append("")

    lines.append("## RAW 陀螺轴能量 (RMS)")
    lines.append("")
    lines.append(f"- 俯仰段: X={rms_pitch[0]:.4f}, Y={rms_pitch[1]:.4f}, Z={rms_pitch[2]:.4f}")
    lines.append(f"- 侧倾段: X={rms_roll[0]:.4f}, Y={rms_roll[1]:.4f}, Z={rms_roll[2]:.4f}")
    lines.append(f"- 平转段: X={rms_yaw[0]:.4f}, Y={rms_yaw[1]:.4f}, Z={rms_yaw[2]:.4f}")
    lines.append("")

    lines.append("## 最佳候选")
    lines.append("")
    lines.append(f"- imu_axis_remap: {list(best.perm)}")
    lines.append(f"- imu_axis_sign: {[float(x) for x in best.signs]}")
    lines.append(f"- euler_map: {best.euler_map.label}")
    lines.append(f"- handedness: {'右手系' if best.handedness > 0 else '左手系'}")
    lines.append(f"- vector_map: {fmt_vector_map(best.perm, best.signs)}")
    lines.append(
        f"- score(total/axis/corr/gravity_err): "
        f"{best.total_score:.4f}/{best.axis_score:.4f}/{best.corr_score:.4f}/{best.gravity_err:.5f}"
    )
    lines.append("")

    lines.append("## 当前仓库配置候选")
    lines.append("")
    lines.append(f"- imu_axis_remap: {list(current.perm)}")
    lines.append(f"- imu_axis_sign: {[float(x) for x in current.signs]}")
    lines.append(f"- euler_map: {current.euler_map.label}")
    lines.append(f"- handedness: {'右手系' if current.handedness > 0 else '左手系'}")
    lines.append(f"- vector_map: {fmt_vector_map(current.perm, current.signs)}")
    lines.append(
        f"- score(total/axis/corr/gravity_err): "
        f"{current.total_score:.4f}/{current.axis_score:.4f}/{current.corr_score:.4f}/{current.gravity_err:.5f}"
    )
    if rank_current is None:
        lines.append("- rank_in_search: 当前配置不满足本次约束，未参与排名")
    else:
        lines.append(f"- rank_in_search: {rank_current}/{len(results)}")
    lines.append("")

    lines.append("## 结论")
    lines.append("")
    if (
        best.perm == current.perm
        and best.signs == current.signs
        and best.euler_map == current.euler_map
    ):
        lines.append("- PASS: 当前仓库配置与最佳候选一致")
    else:
        lines.append("- FAIL: 当前仓库配置与最佳候选不一致")
        lines.append(f"- 分数差: {best.total_score - current.total_score:+.4f}")

    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n报告已保存: {out}")


def main():
    args = parse_args(sys.argv[1:])
    code = run_audit(args)
    sys.exit(code)


if __name__ == "__main__":
    main()
