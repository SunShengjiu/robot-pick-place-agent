# Code as Policies 规划边界

Code as Policies 集成在 `agent/planner.py`，位置位于场景观测和技能执行之间：

```text
自然语言 → planner → 有限 ToolCall → skills.pick_and_place → RobotPort
```

规划器向模型提供场景对象 ID 和有限工具定义。模型可以选择 `pick_and_place` 或 `request_clarification`，但不能生成或执行 Python、关节轨迹、MuJoCo 调用或任意代码。每个工具调用都会对当前 `SceneSnapshot` 做 ID 和参数校验。

当前没有配置模型 API 时使用确定性的本地 fallback，保证 mock、CI 和离线环境可运行。接入 API 时只需注入一个接收 prompt、返回 JSON 的 completion 函数，仍然经过同一套工具白名单和场景校验。

查看计划但不执行：

```bash
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main plan "把红色方块放进蓝色盒子"
```

执行任务时，`run` 复用同一个 planner；规划来源会写入结果的 `policy_source` 字段。

任务 CLI 返回码为：`0` 表示 succeeded，`1` 表示 failed，`2` 表示 uncertain。规划失败不会调用机器人；模拟执行结果会在 evidence 中标记 `execution_mode: "mock"`。
