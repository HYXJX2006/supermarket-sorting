# 团队开发规范

## 1. 开发前同步

```bash
git switch develop
git pull --rebase origin develop
git switch -c feature/<简短名称>
```

每个分支只解决一个主题。不要把实验日志、个人路径、密码、镜像 tar 或训练缓存提交进去。

## 2. 提交规范

使用以下前缀：

```text
feat: 新功能
fix: 修复问题
refactor: 重构
 test: 测试
 docs: 文档
 chore: 构建或环境
 experiment: 实验参数或结果
```

示例：

```text
fix: correct right arm command feedback
feat: add grasp evaluation report
test: validate random five-target planning
docs: update Docker startup guide
```

## 3. Pull Request 要求

PR 目标分支默认是 `develop`，描述至少包含：

- 修改目的；
- 影响文件；
- 运行过的检查命令；
- 是否发送底盘/机械臂/夹爪命令；
- 是否改变正式比赛默认参数；
- 已知风险和回滚方式。

涉及真实动作的 PR 必须明确写出：

```text
动作模式：plan-only / execute
动作范围：底盘 / 左臂 / 右臂 / 夹爪 / 升降柱
速度上限：
超时：
验证结果：
```

## 4. Issue 分类建议

```text
[环境] Docker、GPU、ROS、WSL/GUI
[视觉] YOLO、RGB-D、ArUco、坐标转换
[导航] 观察路线、目标导航、避障
[抓取] IK、deploy、creep、夹爪、lift
[比赛] 任务队列、裁判、完整闭环
[文档] 规则、部署、实验记录
```

Issue 应包含复现方式、日志位置、期望结果和实际结果。

## 5. 安全规则

- 默认先使用 plan-only；
- 不在共享分支直接启动真实动作；
- 不覆盖官方基线文件，除非 PR 明确说明；
- 修改前保留可回滚提交；
- 不提交任何密钥和个人凭据；
- 结论必须区分“静态检查通过”和“仿真真实动作通过”。

