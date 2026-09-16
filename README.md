# robot-pick-place-agent

面向自然语言抓取放置任务的模块化机器人项目。通过独立的机械臂、RGB-D 感知和语言模型适配器，复用任务流程、场景表示与结果检查。

## 当前状态

项目已包含可运行的 mock CLI，并提供可选 MuJoCo 物理验证闭环；模型集成、ROS/PiPER 真机抓放仍在后续阶段。

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

1. 核心数据类型、接口、模拟机器人和最小 CLI。
2. PiPER / ROS 桥接与固定坐标抓取放置。
3. D435i、相机标定、检测和视觉抓放。
4. 模型 API、高层工具调用和真实结果检查。
5. 异常处理、批量测试与可复现部署说明。

## 本地运行

```bash
PYTHONPATH=src python3 -m pytest -q
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main run "把红色方块放进蓝色盒子"
python3 -m pip install -e '.[simulation]'  # 需要 MuJoCo 时
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main simulate
```

MuJoCo 仿真会返回关节方向/限位、接触、抬升和最终落点证据。只有方块由接触夹持并抬离桌面后，`success` 才会为真；固定绑定或直接改写方块位姿不计入验证。

初期只建立有实际内容的模块；完整目录规划见架构文档。

## 上游与许可证

计划集成 PiPER ROS、RealSense ROS、GroundingDINO，并参考 Code as Policies 的提示和 API 组织方式。实际接入时记录上游版本、许可和修改范围。

项目开源许可证待定。
