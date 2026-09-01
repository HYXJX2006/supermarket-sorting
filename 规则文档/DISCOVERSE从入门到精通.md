# DISCOVERSE 全流程教学文档

> 基于项目内所有说明文件整理，涵盖从安装、部署、测试到编写个人项目的完整流程

---

## 目录

1. [项目概述](#1-项目概述)
2. [环境准备](#2-环境准备)
3. [安装步骤](#3-安装步骤)
4. [验证安装](#4-验证安装)
5. [核心概念与架构](#5-核心概念与架构)
6. [快速上手示例](#6-快速上手示例)
7. [编写个人项目](#7-编写个人项目)
8. [常见问题排查](#8-常见问题排查)

---

## 1. 项目概述

### 1.1 什么是 DISCOVERSE？

DISCOVERSE 是一个基于 3D 高斯散射（3DGS）的统一、模块化、开源的 Real2Sim2Real 机器人学习仿真框架，已被 IROS 2025 接收。

### 1.2 核心功能

| 模块 | 功能 | 适用场景 |
|------|------|----------|
| **核心仿真** | MuJoCo 物理引擎集成 | 基础开发、学习 |
| **高保真渲染** | 3D高斯散射渲染 | 视觉仿真、Real2Sim |
| **激光雷达** | Taichi GPU加速的LiDAR仿真 | SLAM、导航研究 |
| **模仿学习** | ACT/Diffusion/RDT等算法 | 机器人技能学习 |
| **硬件集成** | RealSense+ROS支持 | 真实机器人控制 |
| **场景编辑** | MuJoCo XML可视化编辑 | 场景设计、模型调试 |

### 1.3 支持的机器人

- **机械臂**: Airbot Play, ARX L5/X5, IIWA14, Panda, Piper, RM65, UR5e, Xarm7
- **移动机器人**: MMK2, Skyrover, RM2 Car
- **触觉手**: Leap Hand

---

## 2. 环境准备

### 2.1 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04+ / macOS / Windows |
| Python | 3.8+（推荐3.10） |
| CUDA | 11.8+（如需3DGS渲染） |
| GPU | NVIDIA GPU（推荐，用于加速渲染和训练） |
| 内存 | 至少8GB，建议16GB+ |

### 2.2 安装 Git LFS

DISCOVERSE 使用 Git LFS 管理大文件，需先安装：

```bash
# Ubuntu/Debian
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs

# macOS
brew install git-lfs
```

### 2.3 安装 CUDA（可选，用于高保真渲染）

从 [NVIDIA 官网](https://developer.nvidia.com/cuda-toolkit-archive) 安装 CUDA 11.8+，根据显卡驱动选择对应版本。

验证安装：
```bash
nvcc --version
nvidia-smi
```

---

## 3. 安装步骤

### 3.1 克隆仓库

```bash
git clone https://github.com/TATP-233/DISCOVERSE.git
cd DISCOVERSE
```

### 3.2 创建虚拟环境

```bash

#首先查看FAQ,以确定python版本。
conda create -n discoverse python=3.10
conda activate discoverse
```

### 3.3 安装依赖

DISCOVERSE 提供多种安装模式，根据需求选择：

#### 模式1：基础安装（推荐初学者）

```bash
pip install -e .
```

**包含**: MuJoCo、OpenCV、NumPy 等基础依赖

#### 模式2：完整安装（包含所有功能）

```bash
pip install -e ".[act_full, gs, lidar, visualization, xml-editor, hardware]"
```

#### 模式3：按需安装

```bash
# 激光雷达SLAM
pip install -e ".[lidar,visualization]"

# 机械臂模仿学习
pip install -e ".[act_full]"

# 高保真视觉仿真
pip install -e ".[gs]"

# GUI场景编辑
pip install -e ".[xml-editor]"

# 硬件集成
pip install -e ".[hardware]"
```

### 3.4 初始化子模块

```bash
# 查看可用子模块
python scripts/setup_submodules.py --list

# 初始化所有子模块
python scripts/setup_submodules.py --all

# 或初始化特定模块
python scripts/setup_submodules.py --module gaussian-rendering act
```

### 3.5 下载3DGS模型

#### 3.5.1 登录 Hugging Face（必需）

**重要**：使用3D高斯渲染器（GS渲染）时，需要登录 Hugging Face 才能下载模型文件。

```bash
# 登录 Hugging Face
huggingface-cli login

# 或设置环境变量（临时有效）
export HUGGINGFACE_HUB_TOKEN="your_token_here"
```

获取 Token 的方法：
1. 访问 [Hugging Face Settings](https://huggingface.co/settings/tokens)
2. 创建一个新的 Access Token（需要 read 权限）
3. 复制 Token 并粘贴到命令行

#### 3.5.2 设置镜像加速（国内用户推荐）

```bash
# 设置 Hugging Face 镜像（国内用户）
export HF_ENDPOINT="https://hf-mirror.com"
```

#### 3.5.3 模型存储位置

模型会自动下载到 `models/3dgs/` 目录。

#### 3.5.4 离线使用

如果无法访问 Hugging Face，可以手动下载模型文件放到 `models/3dgs/` 目录下。

---

## 4. 验证安装

### 4.1 运行验证脚本

```bash
python scripts/check_installation.py
```

### 4.2 测试基础仿真

```bash
# 启动 Airbot Play 机械臂仿真
python discoverse/robots_env/airbot_play_base.py

# 启动 MMK2 移动机器人仿真
python discoverse/robots_env/mmk2_base.py
```

### 4.3 测试逆运动学

```bash
python examples/mocap_ik/mocap_ik_manipulator.py -r airbot_play -t stack_block
```

### 4.4 测试交互式控制

运行仿真后，可使用以下快捷键：

| 按键 | 功能 |
|------|------|
| `h` | 显示帮助菜单 |
| `F5` | 重新加载 MJCF 场景 |
| `r` | 重置仿真状态 |
| `[`/`]` | 切换相机视角 |
| `Esc` | 切换自由相机模式 |
| `p` | 打印机器人状态信息 |
| `Ctrl+g` | 切换高斯渲染 |
| `Ctrl+d` | 切换深度可视化 |

---

## 5. 核心概念与架构

### 5.1 项目结构

```
DISCOVERSE/
├── discoverse/          # 核心框架代码
│   ├── envs/            # 仿真环境（make_env、simulator）
│   ├── robots_env/      # 机器人环境基类
│   ├── task_base/       # 任务基类
│   ├── universal_manipulation/  # 通用操作模块（IK求解、录制、随机化）
│   ├── robots/          # 机器人运动学求解
│   ├── gaussian_web_renderer/   # 高斯渲染器
│   ├── aigc/            # AI生成工具
│   ├── configs/         # 机器人和任务配置文件
│   │   ├── robots/      # 机器人配置
│   │   └── tasks/       # 任务配置
│   ├── doc/             # 文档
│   ├── docker/          # Docker配置
│   └── utils/           # 工具函数
├── examples/            # 示例代码
│   ├── robots/          # 机器人控制示例
│   ├── tasks_airbot_play/  # Airbot任务示例
│   ├── tasks_mmk2/      # MMK2任务示例
│   ├── tasks_hand_arm/  # 触觉手任务示例
│   ├── universal_tasks/ # 通用任务运行时
│   ├── mocap_ik/        # 逆运动学（mink）示例
│   ├── force_control/   # 力控制示例
│   ├── active_slam/     # 主动SLAM示例
│   ├── sensor_lidar/    # 激光雷达示例
│   ├── 3dmouse/         # 3D鼠标控制示例
│   ├── grasp_gen/       # 抓取生成示例
│   ├── gsplat/          # 高斯散射可视化
│   ├── ros1/            # ROS1集成示例
│   ├── ros2/            # ROS2集成示例
│   └── hardware_sim/    # 硬件仿真示例
├── models/              # 模型资源
│   ├── meshes/          # 网格文件
│   ├── mjcf/            # MuJoCo场景描述
│   │   ├── manipulator/ # 机械臂模型
│   │   ├── mobile_chassis/ # 移动底盘模型
│   │   ├── dex_hand/    # 触觉手模型
│   │   ├── object/      # 物体模型
│   │   └── task_environments/  # 任务环境
│   └── 3dgs/            # 高斯散射模型
├── policies/            # 策略学习
│   ├── Diffusion-Policy/ # Diffusion Policy算法
│   ├── RDT/             # RDT算法
│   ├── RL/              # 强化学习（PPO）
│   ├── dp/              # 另一个Diffusion Policy实现
│   ├── openpi/          # OpenPI算法
│   └── infer.py         # 推理脚本
└── scripts/             # 工具脚本
```

### 5.2 MJCF 场景描述

MJCF（MuJoCo XML Configuration Format）是 MuJoCo 使用的配置文件格式，包含：

- `<worldbody>`: 物理世界定义
- `<body>`: 物体定义（可嵌套）
- `<geom>`: 几何形状（碰撞/视觉）
- `<joint>`: 关节定义
- `<inertial>`: 惯性属性
- `<sensor>`: 传感器
- `<actuator>`: 执行器

### 5.3 BaseConfig 配置类

```python
from discoverse.utils.base_config import BaseConfig

cfg = BaseConfig()
cfg.mjcf_file_path = "path/to/scene.xml"
cfg.timestep = 0.005          # 物理仿真时间步（默认0.005）
cfg.decimation = 2            # 每次step的物理步数（默认2）
cfg.sync = True               # 时间同步（遥操作时设为True）
cfg.headless = False          # 无头模式（无显示时设为True）
cfg.use_gaussian_renderer = False  # 是否使用高斯渲染
cfg.enable_render = True      # 是否启用渲染（默认True）
cfg.max_render_depth = 5.0    # 最大渲染深度（默认5.0）
cfg.render_set = {
    "fps": 24,               # 渲染帧率
    "width": 1280,           # 渲染宽度
    "height": 720,           # 渲染高度
}
cfg.gs_model_dict = {}        # 高斯模型字典
cfg.obs_rgb_cam_id = None     # RGB相机ID（观测用）
cfg.obs_depth_cam_id = None   # 深度相机ID（观测用）
```

### 5.4 环境交互接口

```python
observation, privileged_observation, reward, done, info = env.step(action)
```

### 5.5 Mink IK求解器

DISCOVERSE 使用 Mink 库进行逆运动学求解，支持位置和姿态约束。

**安装依赖**：

```bash
pip install mink
pip install "qpsolvers[quadprog]"
```

**使用示例**：

```python
import mink

# 创建配置
configuration = mink.Configuration(mj_model)

# 定义末端执行器任务
end_effector_task = mink.FrameTask(
    frame_name="endpoint",
    frame_type="site",
    position_cost=100.0,
    orientation_cost=10.0,
    lm_damping=1.0,
)

# 定义姿态任务（保持关节在舒适位置）
posture_task = mink.PostureTask(model=mj_model, cost=1e-2)

# 组合任务
mink_tasks = [end_effector_task, posture_task]

# 设置目标
target_se3 = mink.SE3.from_mocap_name(mj_model, mj_data, "target")
end_effector_task.set_target(target_se3)

# 求解IK
vel = mink.solve_ik(configuration, mink_tasks, dt, solver="quadprog", tolerance=1e-3)
configuration.integrate_inplace(vel, dt)
```

### 5.6 universal_manipulation 模块

`universal_manipulation` 模块提供通用的机器人操作功能：

| 组件 | 功能 |
|------|------|
| `robot_interface.py` | 机器人接口抽象 |
| `gripper_controller.py` | 夹爪控制器 |
| `mink_solver.py` | IK求解器封装 |
| `recorder.py` | 数据录制器 |
| `randomization.py` | 场景随机化 |

---

## 6. 快速上手示例

> ⚠️ **重要提示**：以下每个模块都有对应的依赖要求。如果之前只进行了基础安装，运行前请确保已安装所需依赖。

### 6.1 运行预设任务

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


**运行示例**：

```bash
# Airbot Play 放置咖啡杯
python examples/tasks_airbot_play/cover_cup.py

# MMK2 拾取猕猴桃
python examples/tasks_mmk2/kiwi_pick.py

# Leap Hand 触觉仿真
python examples/robots/leap_hand_env.py
```

### 6.2 选择机器人和任务

**前置依赖安装**：

```bash
# 基础安装（包含MuJoCo等核心依赖）
pip install -e .

# Mink IK求解器（用于逆运动学计算）
pip install mink

# Mink依赖的二次规划求解器（zsh需要用引号）
pip install "qpsolvers[quadprog]"
```

**运行示例**：

```bash
# 使用指定机器人和任务
python3 examples/mocap_ik/mocap_ik_manipulator.py -r arx_l5 -t block_bridge_place

# 查看可用机器人
python3 examples/mocap_ik/mocap_ik_manipulator.py -h
```

**可用机器人**: airbot_play, airbot_play_force, arx_l5, arx_x5, iiwa14, panda, piper, rm65, ur5e, xarm7

**可用任务**: block_bridge_place, close_laptop, cover_cup, open_drawer, peg_in_hole, pick_jujube, place_block, place_coffeecup, place_jujube, place_jujube_coffeecup, place_kiwi_fruit, push_mouse, stack_block

### 6.3 记录仿真数据

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


**运行示例**：

```bash
python3 examples/mocap_ik/mocap_ik_manipulator.py -r airbot_play -t stack_block --record --record-frequency 30
```

### 6.4 启动硬件仿真服务

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：

**运行示例**：

```bash
# 启动RPC服务
cd examples/hardware_sim/server
python server.py --host 0.0.0.0 --port 8890 --ctrl-hz 200

# 运行零位示例（新终端）
cd examples/hardware_sim/example
python real_play_return_zero.py
```

### 6.5 力控制示例

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


**运行示例**：

```bash
# 阻抗控制
python examples/force_control/impedance_control.py

# 关节阻抗控制
python examples/force_control/joint_impedance_control.py
```

### 6.6 主动SLAM示例

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：

**前置依赖安装**：

```bash
# 激光雷达和可视化依赖
pip install -e ".[lidar, visualization]"

# 如果需要使用高斯渲染器，需先登录Hugging Face
# huggingface-cli login
```

**运行示例**：

```bash
# MMK2移动机器人SLAM（默认使用高斯渲染器）
python examples/active_slam/mmk2.py

# 打开底盘模式
python examples/active_slam/mmk2_open_car.py
```

> ⚠️ **注意**：以上示例默认启用高斯渲染器，需要登录 Hugging Face 下载模型。如果不想使用高斯渲染，可修改代码中的 `cfg.use_gaussian_renderer = False`。

### 6.7 激光雷达示例

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


**运行示例**：

```bash
# ROS1激光雷达
python examples/sensor_lidar/mmk2_lidar_ros1.py

# ROS2激光雷达
python examples/sensor_lidar/mmk2_lidar_ros2.py
```

### 6.8 通用任务运行时

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


**运行示例**：

```bash
# 运行通用任务
python examples/universal_tasks/universal_task_runtime.py

# CI/CD测试
python examples/universal_tasks/cicd_testing.py
```

---

## 7. 编写个人项目

> ⚠️ **重要提示**：以下每个模块都有对应的依赖要求。如果之前只进行了基础安装，运行前请确保已安装所需依赖。

### 7.1 创建自定义场景

#### 方法1：使用 XML 可视化编辑器（推荐）

**前置依赖安装**：

```bash
# XML编辑器依赖（PyQt5、PyOpenGL）
pip install -e ".[xml-editor]"

# 初始化XML-Editor子模块（首次使用）
python scripts/setup_submodules.py --module xml-editor
```

**启动可视化编辑器**：

```bash
cd submodules/XML-Editor
python -m xml_editor.main
```

**可视化编辑器功能**：

| 功能 | 操作说明 |
|------|---------|
| **场景导航** | 左键拖动旋转视图，右键拖动平移视图，滚轮缩放 |
| **创建几何体** | 从左侧控制面板拖拽几何体按钮到3D视图中 |
| **选择对象** | 左键点击场景中的对象或在层级树中点击 |
| **变换操作** | 选择平移/旋转/缩放模式，使用Gizmo控制器调整 |
| **编辑属性** | 在右侧属性面板中修改位置、旋转、尺寸、颜色等 |
| **层级管理** | 在左侧层级树中管理对象父子关系和分组 |
| **导入模型** | 文件菜单 → 打开，导入现有MJCF XML文件 |
| **导出场景** | 文件菜单 → 保存/另存为，导出为MJCF XML格式 |

**支持的几何体类型**：
- 盒子（Box）
- 球体（Sphere）
- 圆柱体（Cylinder）
- 胶囊体（Capsule）
- 平面（Plane）

**工作流程示例**：

1. **启动编辑器**：`python -m xml_editor.main`
2. **创建地面**：拖拽"平面"按钮到场景中心
3. **添加物体**：拖拽"盒子"或"球体"到场景中
4. **调整位置**：选择"平移"模式，拖动Gizmo调整位置
5. **设置属性**：在属性面板中修改尺寸、颜色、质量等
6. **保存场景**：文件 → 保存，导出为XML文件
7. **使用场景**：将导出的XML文件放到 `models/mjcf/task_environments/` 目录

#### 方法2：使用 mesh2mjcf 工具

**前置依赖安装**：

```bash
# 基础安装（包含mesh2mjcf所需依赖）
pip install -e .
```

**运行示例**：

```bash
# 将OBJ/STL模型转换为MJCF格式
python scripts/mesh2mjcf.py /path/to/your/model.obj --free_joint -cd --verbose

# 参数说明：
# --free_joint: 添加自由关节（可移动物体）
# -cd: 进行凸分解（非凸物体）
# --verbose: 预览模型
# --mass: 设置质量
# --diaginertia: 设置惯性张量
# --rgba: 设置颜色
```

### 7.2 创建自定义任务

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：


#### 步骤1：创建 MJCF 场景文件

在 `models/mjcf/` 目录下创建场景文件：

```xml
<mujoco model="my_task">
    <compiler meshdir="../meshes" texturedir="../meshes/"/>
    <option gravity="0 0 -9.81"/>
    
    <asset>
        <mesh name="my_object" file="object/my_object.obj"/>
    </asset>
    
    <worldbody>
        <geom name="floor" type="plane" size="5 5 0.05"/>
        
        <!-- 机械臂 -->
        <include file="manipulator/airbot_play/robot.xml"/>
        
        <!-- 目标物体 -->
        <body name="target_object" pos="0.5 0 0.1">
            <joint type="free"/>
            <inertial pos="0 0 0" mass="0.5" diaginertia="0.01 0.01 0.01"/>
            <geom type="mesh" mesh="my_object" rgba="0.8 0.2 0.2 1"/>
        </body>
    </worldbody>
</mujoco>
```

#### 步骤2：创建任务脚本

参考 `examples/tasks_airbot_play/` 下的示例创建任务脚本。`make_env` 函数用于组合机械臂和任务场景生成 MJCF 文件，实际的仿真环境需要继承 `SimulatorBase` 类：

```python
import numpy as np
import os
from discoverse.envs.make_env import make_env
from discoverse.envs.simulator import SimulatorBase
from discoverse.utils.base_config import BaseConfig
from discoverse import DISCOVERSE_ASSETS_DIR

class MyTaskEnv(SimulatorBase):
    def __init__(self, robot_name, task_name):
        # 生成MJCF文件
        mjcf_path = os.path.join(DISCOVERSE_ASSETS_DIR, "mjcf", "tmp", f"{robot_name}_{task_name}.xml")
        env = make_env(robot_name, task_name, mjcf_path)
        env.export_xml(mjcf_path)
        
        # 配置仿真参数
        config = BaseConfig()
        config.mjcf_file_path = mjcf_path
        config.timestep = 0.005
        config.decimation = 2
        config.sync = True
        config.headless = False
        config.use_gaussian_renderer = False
        
        super().__init__(config)
        
        # 获取机械臂自由度
        if robot_name in ["airbot_play", "arx_l5", "arx_x5", "piper", "rm65", "ur5e"]:
            self.arm_dof = 6
        elif robot_name in ["panda", "iiwa14", "xarm7"]:
            self.arm_dof = 7
    
    def post_physics_step(self):
        pass
    
    def getChangedObjectPose(self):
        return {}
    
    def checkTerminated(self):
        return False
    
    def getObservation(self):
        obs = {
            "time": self.mj_data.time,
            "joint_pos": self.mj_data.qpos[:self.arm_dof].tolist(),
            "joint_vel": self.mj_data.qvel[:self.arm_dof].tolist(),
        }
        return obs
    
    def getPrivilegedObservation(self):
        return {}
    
    def getReward(self):
        return 0.0

if __name__ == "__main__":
    env = MyTaskEnv(robot_name="airbot_play", task_name="stack_block")
    obs = env.reset()
    
    for _ in range(1000):
        # 简单的随机动作
        action = np.random.randn(env.arm_dof) * 0.1
        obs, _, reward, done, _ = env.step(action)
        
        if done:
            obs = env.reset()
```

### 7.3 数据生成与训练

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：

```bash
# 基础安装
pip install -e .

# 数据收集依赖（录制和转换数据）
pip install -e ".[data-collection]"

# 选择需要的算法依赖（根据训练需求）
# ACT算法
pip install -e ".[act]"

# Diffusion Policy算法
pip install -e ".[diffusion-policy]"

# RDT算法
pip install -e ".[rdt]"

# 或一次性安装所有机器学习依赖
pip install -e ".[ml]"
```

#### 步骤1：生成数据

```bash
cd scripts
python tasks_data_gen.py --robot_name airbot_play --task_name my_task --track_num 100 --nw 8
```

#### 步骤2：转换数据格式

```bash
# Diffusion Policy - 转换为zarr格式
python3 policies/dp/raw2zarr.py -dir data -tn my_task
```

#### 步骤3：训练模型

```bash
# 使用 policies/train.py 统一入口
python3 policies/train.py act -tn my_task
python3 policies/train.py dp --config-path=configs --config-name=my_task

# Diffusion Policy (policies/Diffusion-Policy)
cd policies/Diffusion-Policy
python train.py --config-name airbot.yaml

# Diffusion Policy (policies/dp)
python3 policies/dp/train_eval.py

# RDT算法
cd policies/RDT
python main.py --config configs/base.yaml

# PPO强化学习（状态输入）
cd policies/RL/sbx/PPO_State
python train.py

# PPO强化学习（视觉输入）
cd policies/RL/sbx/PPO_Vision
python train.py
```

#### 步骤4：推理测试

```bash
# 使用 policies/infer.py 统一入口
python3 policies/infer.py act -tn my_task -rn airbot_play -mts 100 -ts 20241125-110709
python3 policies/infer.py dp --config-path=configs --config-name=my_task

# Diffusion Policy (policies/Diffusion-Policy)
cd policies/Diffusion-Policy
python eval.py --config-name airbot.yaml

# RDT算法
cd policies/RDT
python eval.py --config configs/base.yaml

# PPO强化学习
cd policies/RL/sbx/PPO_State
python inference.py
```

### 7.4 ROS集成

**启动前如请确认是否进入环境，如未安装环境或未进入，请返回参考第二节环境准备**：

```bash
# 基础安装
pip install -e .

# ROS支持依赖
pip install -e ".[ros]"

# 激光雷达依赖（如需使用LiDAR功能）
pip install -e ".[lidar]"

# 硬件集成依赖（如需连接真实硬件）
pip install -e ".[hardware]"
```

#### ROS1

**系统依赖安装**：

```bash
# 初始化ROS环境
source /opt/ros/noetic/setup.bash
```

**运行示例**：

```bash
# Airbot Play 机械臂
python examples/ros1/airbot_play_ros1_joy.py
python examples/ros1/airbot_play_cam_ros1.py

# MMK2 移动机器人
python examples/ros1/mmk2_ros1.py
python examples/ros1/mmk2_ros1_joy.py
python examples/ros1/mmk2_teach_bag_ros1.py

# Tok2 双臂机器人
python examples/ros1/tok2_ros1.py
```

#### ROS2

**系统依赖安装**：

```bash

# 初始化ROS环境
source /opt/ros/jazzy/setup.bash
```

**运行示例**：

```bash
# 启动手柄节点（新终端）
ros2 run joy joy_node

# Airbot Play 机械臂
python3 examples/ros2/airbot_play_ros2.py
python3 examples/ros2/airbot_play_ros2_joy.py

# MMK2 移动机器人
python3 examples/ros2/mmk2_ros2.py
python3 examples/ros2/mmk2_ros2_joy.py

# Tok2 双臂机器人
python3 examples/ros2/tok2_ros2.py
```

---

## 8. 常见问题排查

### 8.1 安装问题

#### CUDA/PyTorch 版本不匹配

```bash
# 安装匹配版本
pip install torch==2.2.1 torchvision==0.17.1 --index-url https://download.pytorch.org/whl/cu118
```

#### 缺少 GLM 头文件

```bash
conda install -c conda-forge glm
export CPATH=$CONDA_PREFIX/include:$CPATH
```

#### Taichi 安装失败

```bash
pip install taichi==1.6.0
```

### 8.2 运行时问题

#### Hugging Face 未登录错误

```
检测到未登录 Hugging Face。请执行 `huggingface-cli login` 或 设置 环境变量 `HUGGINGFACE_HUB_TOKEN` 后重试。
```

**解决方案**：

```bash
# 方法1：登录 Hugging Face
huggingface-cli login

# 方法2：设置环境变量（临时有效）
export HUGGINGFACE_HUB_TOKEN="your_token_here"

# 方法3：禁用高斯渲染器（不需要模型下载）
# 修改代码中的 cfg.use_gaussian_renderer = False
```

获取 Token 的方法：
1. 访问 [Hugging Face Settings](https://huggingface.co/settings/tokens)
2. 创建一个新的 Access Token（需要 read 权限）
3. 复制 Token 并粘贴到命令行

#### ROS2 Python 版本不匹配错误

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so' isn't present on the system.
```

**原因**：conda 环境的 Python 版本与系统 ROS2 安装的 Python 版本不匹配。

**解决方案**：

```bash
# 方法1：退出conda环境，使用系统Python
conda deactivate
python3 examples/sensor_lidar/mmk2_lidar_ros2.py

# 方法2：使用ROS1示例（兼容性更好）
python3 examples/sensor_lidar/mmk2_lidar_ros1.py

# 方法3：创建匹配版本的conda环境
# ROS2 Jazzy 需要 Python 3.12
conda create -n discoverse_ros2 python=3.12
conda activate discoverse_ros2
pip install -e ".[ros, lidar]"
```

**Python版本与ROS版本对应关系**：

| ROS版本 | 要求的Python版本 | 推荐Ubuntu版本 |
|---------|-----------------|---------------|
| ROS1 Noetic | Python 3.8 | Ubuntu 20.04 |
| ROS2 Humble | Python 3.10 | Ubuntu 22.04 |
| ROS2 Jazzy | Python 3.12 | Ubuntu 24.04 |

#### ROS1 rospy 模块找不到错误

```
ModuleNotFoundError: No module named 'rospy'
```

**原因**：conda 环境无法访问系统安装的 ROS1 Python 包。`rospy` 不能通过 pip 安装，必须通过系统包管理器安装。

**解决方案**：

```bash
# 方法1：退出conda环境，使用系统Python（推荐）
conda deactivate

# 确保ROS环境变量已设置
source /opt/ros/noetic/setup.bash

# 运行示例
python3 examples/sensor_lidar/mmk2_lidar_ros1.py

# 方法2：安装ROS1（如果系统未安装）
# Ubuntu 20.04
sudo apt-get install ros-noetic-ros-base

# 添加到bashrc（永久生效）
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 方法3：使用virtualenv替代conda
# virtualenv更兼容系统安装的ROS包
```

**重要**：`rospy` 不能通过 pip 安装！只能通过系统包管理器安装。

| 包名 | 安装方式 |
|------|---------|
| `rospy` | `sudo apt-get install ros-noetic-rospy`（仅Ubuntu 20.04） |
| `rclpy` | `sudo apt-get install ros-jazzy-rclpy`（仅Ubuntu 24.04） |
| `rospkg` | ✅ `pip install rospkg`（可通过pip安装） |
| `catkin-pkg` | ✅ `pip install catkin-pkg`（可通过pip安装） |

**Ubuntu版本与ROS版本对应关系**：

| Ubuntu版本 | 支持的ROS版本 | Python版本 |
|-----------|--------------|-----------|
| 20.04 | ROS1 Noetic, ROS2 Foxy | Python 3.8 |
| 22.04 | ROS2 Humble | Python 3.10 |
| 24.04 | ROS2 Jazzy | Python 3.12 |

**检查系统版本**：
```bash
lsb_release -a
```

> ⚠️ **注意**：如果你的系统是 Ubuntu 24.04，只能使用 ROS2 Jazzy，不能使用 ROS1 Noetic！

#### GLX配置错误

```bash
# 检查显卡模式
prime-select query

# 切换到NVIDIA模式
sudo prime-select nvidia
sudo reboot

# 设置环境变量
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

#### EGL初始化错误

```bash
# 安装Mesa驱动
sudo apt-get install mesa-utils libegl1-mesa-dev libgl1-mesa-glx

# 设置环境变量
export MUJOCO_GL=egl
export PYOPENGL_PLATFORM=egl
```
#### GLUT初始化错误

```bash
#在 Ubuntu/Debian 系统上，你需要安装 freeglut3-dev 包（它包含了运行和开发所需的 GLUT 库）。

sudo apt update
sudo apt install freeglut3-dev

#如果你的系统缺少其他的 OpenGL 开发库，也可以一并安装：
sudo apt install libglu1-mesa-dev mesa-common-dev

#安装完成后，重新运行你的程序：

bash
python -m xml_editor.main
```


#### FFmpeg视频编码错误

```bash
# 更新FFmpeg
conda install -c conda-forge ffmpeg=6.0

# 或降级mediapy
pip install mediapy==1.1.0
```

### 8.3 服务器部署

#### 无头服务器运行

```bash
export MUJOCO_GL=egl
echo "export MUJOCO_GL=egl" >> ~/.bashrc
```

### 8.4 Docker部署

```bash
# 构建镜像
docker build -f discoverse/docker/Dockerfile -t discoverse:latest .

# 运行容器
docker run -dit --rm --name discoverse \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    discoverse:latest

# 设置权限
xhost +local:docker

# 进入容器
docker exec -it discoverse bash
```

---

## 参考资源

- **官方文档**: `discoverse/doc/` 目录下各模块文档
- **论文**: https://arxiv.org/abs/2507.21981
- **Real2Sim**: https://github.com/GuangyuWang99/DISCOVERSE-Real2Sim
- **模型可视化**: https://playcanvas.com/supersplat/editor

---

## 许可证

DISCOVERSE 采用 MIT 许可证，详见 LICENSE 文件。

---

*本文档基于 DISCOVERSE 项目内所有说明文件整理，持续更新中。*
