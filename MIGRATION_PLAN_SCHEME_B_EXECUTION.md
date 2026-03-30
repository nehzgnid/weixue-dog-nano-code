# 方案B迁移执行计划（v2，业务分区版）

- 日期：2026-03-30
- 适用仓库：weixue-dog-nano-code
- 目标结构：/rl、/traditional、/common
- 核心目标：先完成低风险结构迁移，再逐步做深层解耦

---

## 1. 迁移原则（可行性优先）

1. 先收口业务，再细分技术层。
2. 先桥接兼容，再替换引用，不做一次性大爆炸重构。
3. 迁移期间保持新旧路径同时可用。
4. 每个阶段都必须有自动化回归命令和独立回滚点。
5. 自动回归禁止触发硬件动作；硬件测试统一放到人工验收阶段。

---

## 2. 目标目录结构

```text
weixue-dog-nano-code/
├─ README.md
├─ MIGRATION_PLAN_SCHEME_B_EXECUTION.md
├─ docs/
│  ├─ architecture/
│  ├─ deployment/
│  ├─ notes/
│  └─ troubleshooting/
├─ reports/
│  ├─ sim2real/
│  └─ imu/
├─ assets/
│  └─ rl/
├─ legacy/
│  └─ wrappers/
├─ common/
│  ├─ config/
│  │  ├─ robot_config.py
│  │  └─ robot_config.json
│  ├─ motion/
│  │  ├─ kinematics.py
│  │  ├─ gait_generator.py
│  │  └─ balance_controller.py
│  ├─ io/
│  │  ├─ robot_io.py
│  │  ├─ robotio_bridge.py
│  │  └─ stm32_bridge.py
│  ├─ imu/
│  │  ├─ imu_transform.py
│  │  └─ imu_frame_unified.yaml
│  └─ tools/
│     ├─ imu/
│     └─ latency/
├─ traditional/
│  ├─ apps/
│  ├─ simulation/
│  └─ tests/
└─ rl/
   ├─ apps/
   ├─ core/
   ├─ config/
   ├─ tests/
   │  └─ smoke/
   ├─ tools/
   └─ docs/
```

依赖方向约束：

1. 允许：traditional -> common。
2. 允许：rl -> common。
3. 禁止：traditional <-> rl 直接依赖。
4. 禁止：common 依赖 rl 或 traditional。

---

## 3. 分阶段执行计划

### Phase 0：基线冻结（不改业务行为）

目标：记录迁移前可运行基线，后续所有问题可回溯。

```bash
cd /workspace/weixue-dog-nano-code
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*' ! -path './__pycache__/*')
python3 traditional_control_tests/test_imu_readout.py --help
python3 sim2real_jetson_tests/01_env_check.py --help
python3 sim2real_jetson_tests/02_stm32_state_probe.py --help
python3 sim2real_jetson_tests/03_single_servo_safe_test.py --help
python3 sim2real_jetson_tests/04_onnx_latency_smoke_test.py --help
python3 sim2real_jetson_tests/05_onnx_policy_safe_motion_test.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py --help
```

建议提交：baseline-before-business-domain-migration

### Phase 1：创建新目录与包骨架

目标：仅创建目录与 __init__.py，不迁移业务代码。

```bash
cd /workspace/weixue-dog-nano-code
mkdir -p docs/{architecture,deployment,notes,troubleshooting}
mkdir -p reports/{sim2real,imu}
mkdir -p assets/rl legacy/wrappers
mkdir -p common/{config,motion,io,imu,tools/imu,tools/latency}
mkdir -p traditional/{apps,simulation,tests}
mkdir -p rl/{apps,core,config,tests/smoke,tools,docs}
find common traditional rl -type d -print0 | xargs -0 -I{} touch {}/__init__.py
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
```

建议提交：scaffold-business-domain-layout

### Phase 1.5：包管理基线（可编辑安装）

目标：消除对手写 sys.path 注入与工作目录的隐式依赖，统一包发现机制。

实施方式：

1. 在根目录引入标准打包配置（优先 pyproject.toml）。
2. 使用 python3 -m pip install -e . 建立开发态可编辑安装。
3. common、rl、traditional 的导入统一走包路径，不再在业务脚本里写 sys.path.insert。
4. 若仓库已存在 pyproject.toml 或 setup.py，只做增量合并，不做覆盖式重写。
5. 结合现有脚本依赖，补齐 project.dependencies（例如 numpy、pyserial、pyyaml、onnxruntime）。

```bash
cd /workspace/weixue-dog-nano-code
if [ -f pyproject.toml ] || [ -f setup.py ]; then
    echo "Detected existing package config, please merge settings instead of overwrite."
else
cat > pyproject.toml <<'EOF'
[build-system]
requires = ["setuptools>=68", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "weixue-dog-nano-code"
version = "0.1.0"
requires-python = ">=3.9"

[tool.setuptools.packages.find]
include = ["common*", "rl*", "traditional*"]
EOF
fi
python3 -m pip install -e .
python3 - <<'PY'
import common
import rl
import traditional
print('EDITABLE_INSTALL_OK')
PY
```

建议提交：package-management-editable-install-baseline

### Phase 2：建立 common 桥接层

目标：让新路径先可 import，旧路径功能保持原样。

实施方式：

1. 在 common 下创建桥接模块，先通过 import 转发旧实现。
2. 先不删除旧文件，仅补齐新路径可用性。
3. 导入回归通过后，下一阶段再迁移真实文件。
4. 在 common/config/robot_config.py 内提供统一的绝对路径解析入口，禁止业务代码直接裸 open('config.json')。

路径解析参考（示意）：

```python
from pathlib import Path

_HERE = Path(__file__).resolve().parent
PROJECT_ROOT = _HERE.parents[1]

ROBOT_CONFIG_JSON = (_HERE / "robot_config.json").resolve()
WAVEGO_DEPLOY_YAML = (PROJECT_ROOT / "rl" / "config" / "wavego_deploy_config.yaml").resolve()
```

```bash
cd /workspace/weixue-dog-nano-code
python3 - <<'PY'
import importlib
modules = [
    'common.config.robot_config',
    'common.motion.kinematics',
    'common.motion.gait_generator',
    'common.motion.balance_controller',
    'common.io.robot_io',
    'common.io.robotio_bridge',
    'common.io.stm32_bridge',
]
for m in modules:
    importlib.import_module(m)
print('COMMON_BRIDGE_IMPORT_OK')
PY
```

建议提交：common-bridge-ready

### Phase 3：迁移 RL 域

目标：先完成 RL 主链路迁移，保持旧路径兼容壳。

迁移优先级：

1. rl/core：obs_builder.py、safety_guard.py。
2. rl/apps：wavego_inference.py、wavego_keyboard_control.py。
3. rl/config：wavego_deploy_config.yaml。
4. rl/tests/smoke：01~05 脚本。
5. rl/tools：test_latency.py。
6. assets/rl：onnx 与 normalizer 资产。

迁移前置（必须先完成）：

1. 先改导入再移动文件：
    - rl/apps/wavego_inference.py、rl/apps/wavego_keyboard_control.py
    - 禁止继续依赖脚本目录注入 sys.path 的同目录裸导入。
2. 统一为包导入（示例）：
    - from rl.core.obs_builder import ObsBuilder
    - from rl.core.safety_guard import SafetyGuard as SharedSafetyGuard
    - from common.io.robotio_bridge import RobotIOBridge
3. 旧路径兼容壳使用 runpy 转发到新路径主入口，并保证 import 时无硬件副作用。
4. 新路径命令统一从仓库根目录运行，优先使用 python3 -m 包路径 的方式回归。

```bash
cd /workspace/weixue-dog-nano-code
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
python3 -m rl.apps.wavego_inference --help
python3 -m rl.apps.wavego_keyboard_control --help
python3 rl/tests/smoke/01_env_check.py --help
python3 rl/tests/smoke/02_stm32_state_probe.py --help
python3 rl/tests/smoke/03_single_servo_safe_test.py --help
python3 rl/tests/smoke/04_onnx_latency_smoke_test.py --help
python3 rl/tests/smoke/05_onnx_policy_safe_motion_test.py --help
# 兼容壳回归
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py --help
```

建议提交：migrate-rl-domain-with-compat-wrapper

### Phase 4：迁移 traditional 域

目标：迁移传统控制入口、仿真与测试；旧根目录入口保留兼容壳。

迁移优先级：

1. traditional/apps：main_trot.py、control_center_unified.py、servo_control_center.py。
2. traditional/simulation：simulation_*.py。
3. traditional/tests：test_*_in_place.py、test_single_leg_step.py。

```bash
cd /workspace/weixue-dog-nano-code
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
python3 - <<'PY'
import importlib
mods = [
    'traditional.apps.main_trot',
    'traditional.apps.control_center_unified',
    'traditional.apps.servo_control_center',
    'traditional.simulation.simulation_unified',
    'traditional.simulation.simulation_trot_interactive',
    'traditional.simulation.simulation_step_in_place',
    'traditional.tests.test_walk_in_place',
    'traditional.tests.test_trot_in_place',
    'traditional.tests.test_single_leg_step',
]
for m in mods:
    importlib.import_module(m)
print('TRADITIONAL_IMPORT_OK')
PY
# 兼容壳导入检查
python3 - <<'PY'
import importlib
for m in ['main_trot', 'control_center_unified', 'simulation_unified']:
    importlib.import_module(m)
print('LEGACY_WRAPPER_IMPORT_OK')
PY
```

建议提交：migrate-traditional-domain-with-compat-wrapper

### Phase 5：IMU 统一与 common 实质化

目标：统一传统与 RL 的 IMU 坐标变换逻辑，消除双标准。

实施方式：

1. 新增 common/imu/imu_transform.py + imu_frame_unified.yaml。
2. rl/core/obs_builder.py 改为调用 common IMU 变换。
3. common/tools/imu/test_imu_readout.py 与 auto_imu_observation_audit.py 使用同一套配置。

```bash
cd /workspace/weixue-dog-nano-code
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
python3 common/tools/imu/test_imu_readout.py --help
python3 common/tools/imu/auto_imu_observation_audit.py --help
```

建议提交：unify-imu-transform-in-common

### Phase 6：文档、报告、资产收口

目标：根目录瘦身，文档与报告统一归档。

```bash
cd /workspace/weixue-dog-nano-code
find docs -type f | sort
find reports -type f | sort
find assets -type f | sort
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
```

建议提交：consolidate-docs-reports-assets

### Phase 7：兼容壳下线与技术债清理

目标：在新路径稳定运行后，删除历史兼容壳与 legacy 目录，完成收口。

执行前置：

1. 新路径链路完成全量人工验收。
2. 生产或准生产环境连续稳定运行 >= 14 天，无回滚。
3. 兼容壳调用日志为 0 或低于约定阈值。

执行动作：

1. 删除旧路径 runpy 包装器与 legacy/wrappers。
2. 清理文档中旧命令与旧路径引用。
3. 将“兼容壳保留规则”改为“兼容壳已下线”。

```bash
cd /workspace/weixue-dog-nano-code
find legacy/wrappers -type f 2>/dev/null || true
grep -RIn "runpy\.|legacy/wrappers|strategy_deploy_bundle/scripts/wavego_" --include='*.py' . || true
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*')
```

建议提交：cleanup-legacy-wrappers-after-soak

---

## 4. 文件迁移映射（核心清单）

### 4.1 common（共享能力）

1. robot_config.py -> common/config/robot_config.py（MOVE+WRAP）。
2. config.json -> common/config/robot_config.json（MOVE+WRAP）。
3. kinematics_v5.py -> common/motion/kinematics.py（MOVE+WRAP）。
4. gait_generator.py -> common/motion/gait_generator.py（MOVE+WRAP）。
5. balance_controller.py -> common/motion/balance_controller.py（MOVE+WRAP）。
6. robot_io.py -> common/io/robot_io.py（MOVE+WRAP）。
7. sim2real_jetson_tests/robotio_bridge.py -> common/io/robotio_bridge.py（MOVE+WRAP）。
8. sim2real_jetson_tests/strategy_deploy_bundle/scripts/stm32_bridge.py -> common/io/stm32_bridge.py（MOVE+WRAP）。
9. traditional_control_tests/test_imu_readout.py -> common/tools/imu/test_imu_readout.py（MOVE+WRAP）。
10. traditional_control_tests/auto_imu_observation_audit.py -> common/tools/imu/auto_imu_observation_audit.py（MOVE+WRAP）。
11. test_latency_linux.py -> common/tools/latency/test_latency_linux.py（MOVE+WRAP）。
12. test_physical_latency.py -> common/tools/latency/test_physical_latency.py（MOVE+WRAP）。
13. test_led_latency.py -> common/tools/latency/test_led_latency.py（MOVE+WRAP）。

### 4.2 rl（部署主链路）

1. sim2real_jetson_tests/strategy_deploy_bundle/scripts/obs_builder.py -> rl/core/obs_builder.py（MOVE+WRAP）。
2. sim2real_jetson_tests/strategy_deploy_bundle/scripts/safety_guard.py -> rl/core/safety_guard.py（MOVE+WRAP）。
3. sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py -> rl/apps/wavego_inference.py（MOVE+WRAP）。
4. sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py -> rl/apps/wavego_keyboard_control.py（MOVE+WRAP）。
5. sim2real_jetson_tests/strategy_deploy_bundle/scripts/test_latency.py -> rl/tools/test_latency.py（MOVE+WRAP）。
6. sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_deploy_config.yaml -> rl/config/wavego_deploy_config.yaml（MOVE+WRAP）。
7. sim2real_jetson_tests/01_env_check.py -> rl/tests/smoke/01_env_check.py（MOVE+WRAP）。
8. sim2real_jetson_tests/02_stm32_state_probe.py -> rl/tests/smoke/02_stm32_state_probe.py（MOVE+WRAP）。
9. sim2real_jetson_tests/03_single_servo_safe_test.py -> rl/tests/smoke/03_single_servo_safe_test.py（MOVE+WRAP）。
10. sim2real_jetson_tests/04_onnx_latency_smoke_test.py -> rl/tests/smoke/04_onnx_latency_smoke_test.py（MOVE+WRAP）。
11. sim2real_jetson_tests/05_onnx_policy_safe_motion_test.py -> rl/tests/smoke/05_onnx_policy_safe_motion_test.py（MOVE+WRAP）。
12. sim2real_jetson_tests/test_imu_alignment.py -> rl/tests/test_imu_alignment.py（MOVE+WRAP）。
13. sim2real_jetson_tests/strategy_deploy_bundle/config/wavego_policy.onnx 与 normalizer_stats.npz -> assets/rl/（MOVE）。

### 4.3 traditional（传统控制主链路）

1. main_trot.py -> traditional/apps/main_trot.py（MOVE+WRAP）。
2. control_center_unified.py -> traditional/apps/control_center_unified.py（MOVE+WRAP）。
3. servo_control_center.py -> traditional/apps/servo_control_center.py（MOVE+WRAP）。
4. simulation_unified.py -> traditional/simulation/simulation_unified.py（MOVE+WRAP）。
5. simulation_trot_interactive.py -> traditional/simulation/simulation_trot_interactive.py（MOVE+WRAP）。
6. simulation_step_in_place.py -> traditional/simulation/simulation_step_in_place.py（MOVE+WRAP）。
7. simulation_ik_v5.py -> traditional/simulation/simulation_ik_v5.py（MOVE+WRAP）。
8. test_walk_in_place.py -> traditional/tests/test_walk_in_place.py（MOVE+WRAP）。
9. test_trot_in_place.py -> traditional/tests/test_trot_in_place.py（MOVE+WRAP）。
10. test_single_leg_step.py -> traditional/tests/test_single_leg_step.py（MOVE+WRAP）。

### 4.4 docs 与 reports

1. API.md -> docs/architecture/API.md。
2. CODE_DOCUMENTATION.md -> docs/architecture/CODE_DOCUMENTATION.md。
3. README_IMPROVED.md -> docs/architecture/README_IMPROVED_ARCHIVE.md。
4. sim2real_jetson_tests/*.md（测试报告）-> reports/sim2real/。
5. traditional_control_tests/IMU_*REPORT*.md -> reports/imu/。

---

## 5. 自动回归门禁（按阶段启用，避免前置路径误报）

### 5.1 全阶段通用必跑

```bash
cd /workspace/weixue-dog-nano-code
python3 -m py_compile $(find . -type f -name '*.py' ! -path './.git/*' ! -path './__pycache__/*')
```

```bash
cd /workspace/weixue-dog-nano-code
python3 -m pip show ruff >/dev/null 2>&1 || python3 -m pip install ruff
ruff check common rl traditional --select F401,F821,E9
```

### 5.2 Phase 0 到 Phase 2 门禁（仅旧路径）

```bash
cd /workspace/weixue-dog-nano-code
python3 traditional_control_tests/test_imu_readout.py --help
python3 sim2real_jetson_tests/01_env_check.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py --help
```

### 5.3 Phase 3 到 Phase 4 门禁（新路径 + 兼容壳）

```bash
cd /workspace/weixue-dog-nano-code
python3 -m rl.apps.wavego_inference --help
python3 -m rl.apps.wavego_keyboard_control --help
python3 rl/tests/smoke/01_env_check.py --help
python3 rl/tests/smoke/02_stm32_state_probe.py --help
python3 rl/tests/smoke/03_single_servo_safe_test.py --help
python3 rl/tests/smoke/04_onnx_latency_smoke_test.py --help
python3 rl/tests/smoke/05_onnx_policy_safe_motion_test.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_inference.py --help
python3 sim2real_jetson_tests/strategy_deploy_bundle/scripts/wavego_keyboard_control.py --help
```

### 5.4 Phase 5 及以后门禁（IMU 共用实现）

```bash
cd /workspace/weixue-dog-nano-code
python3 common/tools/imu/test_imu_readout.py --help
python3 common/tools/imu/auto_imu_observation_audit.py --help
python3 traditional_control_tests/test_imu_readout.py --help
```

### 5.5 依赖方向检查（建议从 Phase 2 开始每阶段执行）

```bash
cd /workspace/weixue-dog-nano-code
# common 不得依赖业务域
grep -RIn "from rl\.|import rl\.|from traditional\.|import traditional\." --include='*.py' common || true
# rl 不得依赖 traditional
grep -RIn "from traditional\.|import traditional\." --include='*.py' rl || true
# traditional 不得依赖 rl
grep -RIn "from rl\.|import rl\." --include='*.py' traditional || true
```

### 5.6 配置路径检查（禁止裸相对路径）

```bash
cd /workspace/weixue-dog-nano-code
if grep -REn "open\(['\"](config\.json|wavego_deploy_config\.yaml)['\"]\)" --include='*.py' common rl traditional; then
    echo "FOUND_RELATIVE_CONFIG_PATH: use common.config absolute path API"
    exit 1
fi
```

---

## 6. 人工验收测试（硬件安全前提）

1. 串口状态读取：rl/tests/smoke/02_stm32_state_probe.py。
2. 单舵机安全微动：rl/tests/smoke/03_single_servo_safe_test.py。
3. RL dry-run 与低速命令联机：python3 -m rl.apps.wavego_inference。
4. 传统步态与 UI 手工确认：traditional/tests/*、traditional/apps/*。
5. IMU 一致性确认：common/tools/imu/auto_imu_observation_audit.py + test_imu_readout.py。

---

## 7. 回滚策略

1. 每个 Phase 独立提交，失败时只回滚当前阶段。
2. 兼容壳在全部人工验收通过前不得删除。
3. 删除重复模型前先做哈希比对。
4. 遇到链路中断时，优先恢复可运行入口，再继续迁移。

---

## 8. 完成标准（DoD）

1. 自动回归全绿：语法、导入、CLI smoke、依赖方向检查全部通过。
2. 新旧路径并行可用：新路径可运行，旧路径兼容壳仍可用。
3. RL 与传统主链路都通过最小人工验收。
4. IMU 坐标链路只有一套标准，审计结论与配置一致。
5. 根目录仅保留稳定入口与总文档，不再堆叠临时脚本与历史报告。
6. Phase 7 完成后，legacy/wrappers 与旧兼容壳已下线，且稳定运行 >= 14 天。

---

## 9. 执行要求（给后续AI/工程师）

1. 严格按 Phase 顺序执行，不跳步。
2. 当前阶段未通过，不进入下一阶段。
3. 每次改动都记录命令、结果、问题与修复。
4. 硬件动作默认先 dry-run 或最小幅度。
5. 迁移期间优先保证“可运行”，其次才是“更优雅”。
6. 兼容壳文件必须做到 import 无副作用，硬件动作只能在 __main__ 入口触发。
7. Python 业务代码禁止新增 sys.path.insert；统一使用已安装包导入。
8. 配置文件读取统一经 common.config 暴露的绝对路径入口，不允许裸相对路径。
