# robot-pick-place-agent

面向自然语言抓取放置任务的模块化机器人项目。通过独立的机械臂、RGB-D 感知和语言模型适配器，复用任务流程、场景表示与结果检查。

## 当前状态

当前主演示为 **M1：完整 PiPER 六轴机械臂＋末端夹爪的 MuJoCo 运动验收**。
已接入官方 humble 固定版本模型，完成逐关节运动、返回指定姿态、夹爪开合。
模型按当前 URDF 适配，具体硬件/固件匹配仍待确认。IK、整臂物理抓放、实际模型 API＋CaP、相机与真机迁移尚未完成。

[完整机械臂截图](artifacts/piper_m1/home_hold.png) ·
[连续运动视频](artifacts/piper_m1/piper_m1_continuous.mp4) ·
[M1 核对报告与关节映射](docs/piper-m1.md) · [来源与版本](UPSTREAM.md)

## 首个部署目标

- 硬件：PiPER 单臂及夹爪、RealSense D435i、单张 RTX 5080。
- 环境：Ubuntu 22.04、ROS 2 Humble；语言模型优先通过 API 调用。
- 任务：输入“把红色方块放进蓝色盒子”，识别真实物体，执行抓取放置并检查结果。
- 第一版场景：已知刚性物体、固定桌面与相机、受限工作区域。

## 架构与复用

完整设计见 [仓库结构与接口边界](docs/architecture.md)。

| 模块 | 职责 |
|---|---|
| `core` | 设备无关的数据类型、接口和几何规则 |
| `perception` | 检测、深度定位与场景快照 |
| `skills` | 抓取放置流程及结果检查 |
| `agent` | 自然语言理解、高层工具选择与反馈 |
| `adapters` | 具体机器人、相机、检测模型和语言模型 |
| `runtime` / `cli` | 模块组装、任务状态、日志与命令行 |

设备参数与现场标定通过配置提供。核心逻辑不直接依赖 ROS、PiPER SDK 或 GPU 库，使模拟设备、离线回放和真实设备可以遵守同一接口。

## 实施顺序

1. M1：官方 PiPER＋夹爪模型、逐关节与开合验收（已完成仿真验证）。
2. M2：已知 state 坐标下整臂 IK、避碰和物理抓放，记录成功/失败证据。
3. M3：实际模型 API＋CaP，通过受控技能接口驱动同一 PiPER 流程。
4. M4：相机观测、PiPER ROS 2 / D435i 适配与现场标定验收。

## 本地运行

```bash
python3 -m pip install --user 'setuptools>=68'
python3 -m pip install -e '.[simulation-media]'
MUJOCO_GL=egl robot-agent piper-m1  # 生成完整机械臂视频，需要 ffmpeg
robot-agent piper-m1 --no-video --output artifacts/piper_m1_headless
PYTHONPATH=src python3 -m pytest -q
```

现有 mock 与早期接触测试入口继续保留，仅用于各自的流程/接触检查：

```bash
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main run "把红色方块放进蓝色盒子"
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main plan "把红色方块放进蓝色盒子"
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main simulate  # 旧简化夹爪，不是 PiPER
```

第一轮正确性约定：`run` 的退出码为 `0=succeeded`、`1=failed`、`2=uncertain`；规划不明确、否定或不支持的指令不会调用机器人。结果 JSON 会区分 `mock_flow` 与设备反馈，Mock 流程成功不代表物理抓放成功。

PiPER M1 返回明确的里程碑范围和关节执行证据，不声称抓放或 CaP 成功。
后续 M2 最终成功必须同时验证夹持、持续抬升、运输、释放、夹爪撤离和物体稳定落入容器；固定绑定或执行中改写物体位姿不计入验证。

初期只建立有实际内容的模块；完整目录规划见架构文档。

## 上游与许可证

已导入 PiPER ROS humble 的官方模型和网格，固定提交及适配内容见 [UPSTREAM.md](UPSTREAM.md)。
后续接入 RealSense ROS、相机观测和实际模型 API，参考 Code as Policies 的提示、任务程序与技能 API 组织。

项目开源许可证待定。
