# 环境与启动说明

## 运行分工

```text
Server：MuJoCo、3DGS、ROS 传感器、任务消息和控制接口
Client：检测器、任务编排器、抓取规划器和动作 worker
```

当前 Server/Client Docker 镜像为团队共享制品，不纳入本仓库。

## 环境变量基线

```dotenv
ROS_DOMAIN_ID=99
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SUPERMARKET_RANDOMIZE=1
SUPERMARKET_RANDOMIZE_OBSTACLES=1
SUPERMARKET_TASK_COUNT=5
SUPERMARKET_USE_GS=1
SUPERMARKET_ENABLE_RENDER=1
SUPERMARKET_HEADLESS=0
TORCH_CUDA_ARCH_LIST=8.9
MAX_JOBS=2
```

实际运行前请根据本机 WSL、显示服务和镜像 tag 调整，不要把个人 `.env` 提交到 Git。

## 验证层级

```text
1. 静态检查：AST/py_compile
2. ROS 只读检查：topic、odom、joint_states、相机和雷达
3. plan-only：不发送动作命令
4. 单动作 execute：明确动作名、超时和速度
5. 单商品闭环
6. 随机 5 目标完整闭环
```

任何阶段失败都应记录命令、日志、提交 SHA 和结果，不要只记录“成功/失败”。
