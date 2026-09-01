# supermarket-sorting

揭榜挂帅智能超市分拣机器人项目。

当前 `develop` 是团队联调分支，包含随机商品场景、随机 5 目标任务队列、ROS 2/MuJoCo/3DGS 仿真、YOLO 多类别感知、目标导航规划和抓取动作编排。

> 当前版本定位：**随机 5 目标任务规划与联调开发版**。真实右臂抓取、物体保持、配送放置和 5 目标完整闭环仍需继续验证，不能视为最终比赛交付版。

## 快速开始

### 获取代码

```bash
git clone -b develop https://github.com/HYXJX2006/supermarket-sorting.git
cd supermarket-sorting
```

### 当前随机 5 目标入口

主要入口位于 `baseline/`：

- `random_server_bootstrap.py`：随机布局与随机 5 目标任务选择；
- `competition_executor.py`：目标识别、货位关联、导航目标和计划编排；
- `multiclass_detect.py`：9 类 YOLO RGB-D 检测；
- `single_item_executor.py`：单目标动作状态机；
- `grasp_planner.py`：目标坐标、抓取偏置和 plan-only IK；
- `grasp_evaluator.py`：只读关节/FK 评估；
- `random_plan_only.sh`：随机 5 目标 plan-only 联调入口。

Docker 镜像不在 Git 仓库中。请从团队约定的镜像仓库或共享存储获取，并按实际镜像 tag 配置启动脚本。

## 分支策略

```text
main                         稳定可交付版本
└── develop                  团队日常集成和联调
    ├── feature/<name>       新功能
    ├── fix/<name>           问题修复
    └── experiment/<name>    实验和参数标定
```

日常开发流程：

```bash
git switch develop
git pull --rebase origin develop
git switch -c feature/your-change
# 修改和测试
git add <明确的文件>
git commit -m "feat: describe the change"
git push -u origin feature/your-change
```

然后通过 Pull Request 合并到 `develop`。不要直接向 `main` 推送；涉及机械臂、底盘或比赛流程的修改必须附带验证命令和结果。

## 当前验证边界

已具备或已有代码支持：

- 45 个商品的随机换位场景；
- 随机 5 个任务目标；
- 随机障碍物生成；
- YOLO 多类别检测和短标签结果图；
- 多帧候选确认、货架/层位关联；
- 目标专属导航规划；
- plan-only IK；
- safe_pose、deploy、creep、close、lift、retreat、deliver、place 状态机框架。

仍需继续验收：

- 右臂真实关节跟随和 deploy 精度；
- 桶装薯片的闭爪和 lift 保持；
- retreat 后物体是否跟随；
- 障碍区配送和 place；
- 裁判结果；
- 随机 5 目标完整连续闭环。

## 大文件和敏感信息

以下内容不提交到 Git：

- Docker 镜像 tar、ISO、EXE；
- YOLO 权重和训练缓存；
- 3DGS 点云/模型二进制；
- `debug_data/`、日志、缓存、备份；
- 密码、Token、SSH 私钥和本地密钥。

对应的下载地址、SHA256、镜像 Digest 和实验摘要应写入文档或单独的团队制品存储。

## 协作文档

- [团队开发规范](CONTRIBUTING.md)
- [当前项目进度](docs/team-progress.md)
- [环境与启动说明](docs/setup.md)
