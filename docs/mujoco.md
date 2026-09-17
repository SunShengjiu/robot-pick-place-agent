# MuJoCo 物理验证

> 历史简化夹爪夹持实验，仅保留为接触测试；不是当前项目主演示，也不是 PiPER 抓放验收。
> 完整机械臂入口为 `robot-agent piper-m1`，见 [PiPER M1](piper-m1.md)。下文保留早期实验记录。

仓库提供一个无界面的 MuJoCo 固定场景，用于验证“红色方块放入蓝色盒子”的接触和重力行为。场景包含桌面、自由刚体方块、带围栏的盒子和两个由位置执行器驱动的夹爪指。抓取流程通过 `RobotPort` 的移动和夹爪接口执行；方块位置来自 MuJoCo 状态，放置后等待重力稳定，再检查方块是否落在盒内。代码不会把方块移动到夹爪上，也没有 weld 或固定绑定。

安装可选依赖：

```bash
python3 -m pip install -e '.[simulation]'
```

运行无头仿真：

```bash
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main simulate
# 或
PYTHONPATH=src python3 examples/mujoco_pick_place.py
```

两种观测模式必须显式选择：

```bash
# 控制调试：读取 MuJoCo body 位姿
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main simulate --observation state
# 感知试验：只读取渲染 RGB 并做颜色检测
PYTHONPATH=src python3 -m robot_pick_place_agent.cli.main simulate --observation camera
```

`state` 的场景来源是 `mujoco_sim_state`，允许输出 `cube_position` 并用于抬升/落点判定；`camera` 的来源是 `mujoco_camera_rgb`，只能从像素得到对象估计，结果不会包含模拟器直接提供的物体位姿，当前也不会声称已完成深度/抬升判定。

`mujoco` 未安装时，核心和 mock 模式仍可运行，`simulate` 会返回安装提示。仿真结果只验证该简化夹爪和固定场景的物理闭环，不能替代 PiPER、真实夹爪或现场标定验收。

仓库没有复制 PiPER 厂商模型资产。`MujocoRobot` 支持 `xml_path` 加载外部 MJCF；接入真实 PiPER 模型前，需要把模型中的末端、左右夹指关节和方块/盒子 body 名称映射到适配器，再运行同一组判据。当前 CLI 的 `--xml` 参数会明确提示这个映射尚未完成，不会误把未知模型当作已验证。

当前这版运行结果会同时报告 `grasp_contact`、`lifted` 和 `success`。在当前简化夹爪参数下，已观察到两指与方块接触，但摩擦夹持尚未把方块抬离桌面，因此 `success=false`；这是真实物理失败证据，不会被流程层标记为成功。下一步应调节夹指形状、接触摩擦和控制增益，再以 `lifted=true` 作为抓取通过条件。
