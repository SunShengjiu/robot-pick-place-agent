# PiPER M2a：TCP 位姿 IK、轨迹和碰撞预检

M2a 在完整 PiPER 模型上实现并验收末端位姿控制，但还没有抓放物体。
`PiperMujocoRobot.move_to(Pose)` 先在 scratch `MjData` 上求解六关节 IK，
再对从当前关节状态到解的连续五次轨迹做限位和碰撞预检，最后由电机输入和
`mj_step` 积分实际关节运动。执行中不写方块状态，也不绑定物体；本场景没有物体。

[M2a 连续视频](../artifacts/piper_m2a/piper_m2a_continuous.mp4) ·
[结果 JSON](../artifacts/piper_m2a/result.json) ·
[关节/TCP 记录](../artifacts/piper_m2a/tcp_joint_trace.csv) ·
[M2a 适配模型](../artifacts/piper_m2a/adapted_model.xml)

## TCP 定义

当前候选模型将 TCP 定义为 `link6` 局部坐标 `[0, 0, 0.125]` m，方向与
`link6` 坐标系相同。它位于官方 `gripper_base` 安装区域内，用于 M2a 位姿
验证；不是实物 TCP 标定值。`base` 与官方 `base_link` 重合，位置单位 m，
角度单位 rad，应用层四元数顺序为 xyzw。适配器负责转换 MuJoCo 内部 wxyz。

实物仍需确认法兰安装方向、夹爪指尖中心、TCP 方向和工具长度；这些确认前不能
把仿真 TCP 直接用于真机。M2a 通过 `get_state()["pose"]` 读取实际 MuJoCo
site 反馈，而不是把 IK 输入当作到达证据。

## IK 和执行

`solve_ik` 使用位置/姿态几何残差的阻尼最小二乘迭代：

- `mj_jacSite` 提供实际 TCP 的位置与旋转雅可比；阻尼项抑制奇异点。
- 每步关节增量限制为 0.08 rad，并裁剪到当前 URDF 限位；默认最多 300 次迭代。
- 位置和姿态残差阈值分别为 0.2 mm 与 0.002 rad。默认 seed 是当前实测关节状态，
  因而连续目标会沿当前分支求解。
- IK 只在 scratch 数据上更新，live `qpos` 和 `data.time` 不变；执行成功后才
  通过 `_execute` 发送电机控制。

轨迹是当前状态到 IK 解的五次时间标定曲线，J1–J6 速度上限 0.5 rad/s、
加速度上限 1 rad/s²；每个物理步由 PD＋模型 bias 补偿的仿真电机驱动。
`check_joint_trajectory` 在 121 个均匀采样点检查关节限位和所有启用碰撞几何，
碰撞或越限时在执行前返回失败。M2a 轨迹记录每 1 ms 物理步；本次最大相邻关节
采样差 0.000464 rad，最大穿透 0，最大接触数 0。采样是预检离散近似，不是任意
连续路径的数学证明；M2 需要按任务路径提高采样和现场安全裕量。

碰撞报告包含几何名称和穿透深度。官方两个夹指网格在机械闭合止挡有亚微米
CAD 包络重叠，闭合目标的这一对接触被标为预期止挡；桌面、非相邻自碰撞及其他
夹指路径仍会拒绝。碰撞网格为官方 STL 凸包，精度限制见 [M1 报告](piper-m1.md)。

## 三个 TCP 目标

验收脚本先用独立的正向反馈生成三个已知可达的 TCP target Pose，然后只把这些
Pose 交给 `move_to`。这不会绕过关节执行：目标关节只用于生成测试姿态，执行阶段
仍通过 IK 重新求解。

| 目标 | TCP 位置 (m) | IK 位置误差 | IK 姿态误差 | 实际反馈位置误差 | 实际反馈姿态误差 |
|---:|---|---:|---:|---:|---:|
| 1 | (0.309954, 0.052247, 0.331060) | 0.0397 mm | 0.000082 rad | 0.0398 mm | 0.000082 rad |
| 2 | (0.320526, −0.073552, 0.313958) | 0.0691 mm | 0.000073 rad | 0.0693 mm | 0.000073 rad |
| 3 | (0.277691, 0.036446, 0.294614) | 0.1621 mm | 0.000056 rad | 0.1620 mm | 0.000056 rad |

三个目标的 IK 解、预检采样和 actual TCP Pose 均写入 `result.json`。每个目标
执行后从 `get_state()` 读取 site 反馈，再独立计算位置和姿态误差；三者均低于
验收阈值并返回 `target_reached`。脚本还验证一条会穿过桌面的关节轨迹被拒绝，
以及 `[2, 2, 2]` m 不可达位姿被 IK 拒绝。

## 运行和验证

```bash
python3 -m pip install --user 'setuptools>=68'
python3 -m pip install -e '.[simulation-media]'
MUJOCO_GL=egl robot-agent piper-m2a
# 无视频快速验收
PYTHONPATH=src python3 examples/piper_m2a.py --no-video --output artifacts/piper_m2a_headless
PYTHONPATH=src python3 -m pytest tests/test_piper_m2a.py -q
```

本次视频为 6.44 秒、161 帧、25 fps，连续渲染同一 `data.time`，无阶段重置或
姿态传送。状态来源明确为 `mujoco_sim_state`。M2a 不启动 ROS/CAN/D435i，
不调用模型 API/CaP，不报告 physical pick/place success。

## 未解决项

M2a 仍使用候选当前 URDF、未标定 TCP 和仿真 PD/努力限制；实物型号、固件零位、
工具安装、质量惯量、关节控制器和碰撞裕量需现场确认。下一步 M2 应在已知 state
物体和容器坐标下复用这些 IK/轨迹/碰撞接口，执行接近、夹持、抬升、运输、释放、
撤离及稳定落入，并分别保存成功与失败证据。M3 才接入实际模型 API＋受控 CaP；
M4 再接 D435i 和 ROS 2 真机适配。不能把当前三个空场景 TCP 目标外推为抓放成功。
