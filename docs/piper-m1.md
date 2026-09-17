# PiPER M1：整臂与夹爪运动

M1 已完成：官方 PiPER 几何、固定基座、六转动关节和末端双滑动夹指在原生
MuJoCo 中加载，通过动力学执行器逐关节运动、回到指定关节姿态并开合夹爪。
此交付只验证整臂关节控制，不代表 IK、物理抓放、CaP 或真机已完成。

主演示：[连续视频](../artifacts/piper_m1/piper_m1_continuous.mp4)、
[完整机械臂截图](../artifacts/piper_m1/home_hold.png)、
[组合姿态截图](../artifacts/piper_m1/specified_pose_hold.png)、
[闭爪截图](../artifacts/piper_m1/gripper_closed_hold.png)、
[开爪截图](../artifacts/piper_m1/gripper_open_hold.png)。

## 运行

```bash
python3 -m pip install --user 'setuptools>=68'
python3 -m pip install -e '.[simulation-media]'
# 需要系统 PATH 中可用的 ffmpeg；Ubuntu 可用 sudo apt install ffmpeg
MUJOCO_GL=egl robot-agent piper-m1
# 等价源码入口
MUJOCO_GL=egl PYTHONPATH=src python3 examples/piper_m1.py
# 不创建渲染上下文，不需要 ffmpeg/Pillow
PYTHONPATH=src python3 examples/piper_m1.py --no-video --output artifacts/piper_m1_headless
PYTHONPATH=src python3 -m pytest tests/test_piper_m1.py -q
```

验证环境为 Python 3.10.12、MuJoCo 3.13.0，视频使用 EGL 和 ffmpeg。
M1 不启动 ROS、CAN、真机、模型 API 或 D435i。`--output` 可指定其他输出目录。
安装包包含固定版本模型资源，不依赖 `/tmp` 中的上游 checkout。
Ubuntu 22.04 系统自带的 setuptools 59.6 在本环境曾错误生成空的 `UNKNOWN` 包；
已升级至 84.0 并验证实际 wheel 包含资源。若使用隔离虚拟环境，可在环境内升级，省略 `--user`。

## 来源与适配决策

上游为 `agilexrobotics/piper_ros` 的 `humble` 分支，固定提交
`017ffefa64511bc6325bd77ddc4e16065c152051`。完整来源、许可证与文件摘要见
[UPSTREAM.md](../UPSTREAM.md) 和 `assets/piper/source.json`。

官方 README 指定：固件早于 `S-V1.6-3` 使用旧 URDF，之后使用当前 URDF，
J2/J3 零位坐标系偏移 2°。实物固件尚未确认，本次明确采用当前带原装夹爪 URDF
作为候选模型，不声称完全匹配用户设备。

原始 MJCF 的 J2/J3 变换仍接近旧版零位，且质量惯量与同提交当前 URDF 不一致。
选择当前 URDF 作为一致的运动学/惯性依据，而非混用两套参数。

| 核对项 | 原始 MJCF | 本次适配 |
|---|---|---|
| 基座 | 世界上的 mesh | 命名固定 `base_link`，无 freejoint |
| J2 零位 pitch | 约 −0.10095 rad | 当前 URDF −0.1359 rad |
| J3 零位 yaw | 约 −1.759 rad | 当前 URDF −1.7939 rad |
| J1 上限 | joint 2.168，actuator 2.618 | 统一 URDF 2.618 rad |
| J3 下限 | joint −2.967，actuator −2.697 | URDF −2.967 rad |
| J4 控制范围 | ±1.832，超出关节 ±1.745 | 命令校验 ±1.745 rad |
| J6 控制范围 | ±π，超出关节 ±2.0944 | 命令校验 ±2.0944 rad |
| 每指行程 | joint 35 mm，actuator 47.5 mm | 每指 35 mm；总开度 70 mm |
| 质量惯量 | 动态体合计约 1.378 kg，夹爪基座并入 link6 | 每个 link 使用当前 URDF，独立固定 gripper_base |
| 控制 | 高增益位置伺服，最高示例力矩 ±20000 | 独立标注的仿真 PD＋动力学 bias 补偿和限幅电机 |

适配通过 `piper_model.py` 生成，保留原始资源不变。新增桌面、灯光、暂定 TCP site，
将显示网格与碰撞网格分开。没有 mocap、weld、末端平移自由度或绑定物体。
仅初始化时写关节初态；执行过程只更新电机控制输入，由 `mj_step` 积分关节状态。

## 关节映射

角度单位 rad，滑动单位 m；正方向为各关节局部轴的右手方向。
它们不是世界坐标的同一方向。J1–J6 对应官方同名关节，无额外符号翻转。

| API 顺序 / qpos | 关节 | 父 → 子 | 类型 / 局部轴 | 范围 | 执行器 |
|---:|---|---|---|---|---|
| 0 | joint1 | base_link → link1 | hinge / +Z | [−2.618, 2.618] | joint1_motor |
| 1 | joint2 | link1 → link2 | hinge / +Z | [0, 3.14] | joint2_motor |
| 2 | joint3 | link2 → link3 | hinge / +Z | [−2.967, 0] | joint3_motor |
| 3 | joint4 | link3 → link4 | hinge / +Z | [−1.745, 1.745] | joint4_motor |
| 4 | joint5 | link4 → link5 | hinge / +Z | [−1.22, 1.22] | joint5_motor |
| 5 | joint6 | link5 → link6 | hinge / +Z | [−2.0944, 2.0944] | joint6_motor |
| 6 | joint7 | gripper_base → link7 | slide / +Z | [0, 0.035] | joint7_motor |
| 7 | joint8 | gripper_base → link8 | slide / −Z | [−0.035, 0] | joint8_motor |

`gripper_base` 固定安装到 `link6`，无额外运动自由度。
统一夹爪输入是两指总名义开度 `w`：`q7=w/2`、`q8=−w/2`；
正 `w` 为张开，范围 0–0.07 m。两个夹指在 flange 坐标中沿相反 Y 方向移动。
不能把 ROS 消息中的单指位置直接当成总开度。上游 README 提到实物 80 mm，
与该 URDF 的 70 mm 不一致；当前明确拒绝 80 mm 输入，等待实物核对后再改映射。

全零关节表示选定 URDF 的零位，不是本次启动姿态。演示初始/返回姿态为
`[0, 0.8, −0.7, 0, 0.3, 0]`，组合姿态为
`[0.35, 1.05, −1.05, 0.25, 0.5, −0.35]`。
所有逐关节测试从返回姿态出发，只对被测关节增加 0.25 rad，再返回。

## 质量惯量、碰撞与 TCP

| link | 当前 URDF 质量 kg |
|---|---:|
| base_link | 1.020 |
| link1 | 0.710 |
| link2 | 1.160 |
| link3 | 0.500 |
| link4 | 0.380 |
| link5 | 0.383 |
| link6 | 0.007 |
| gripper_base | 0.450 |
| link7 / link8 | 各 0.025 |

总计 4.66 kg，去除固定基座后的运动部分为 3.64 kg。惯性张量直接取 URDF，
经 MuJoCo 主轴分解后独立重构核对，所有张量正定。**这只是来源一致性检查，
不是实测动力学认证**：例如 link6 仅 7 g，但 Ixx 约 0.001521 kg·m²，
对应回转半径约 0.47 m，与小法兰几何尺度不一致，需要厂商/实物核实。

显示使用官方原始 STL。碰撞仍使用逐 link STL 的 MuJoCo 凸包，不能认为是精确
凹网格碰撞；未进行碰撞分解。基座与 link1 的轴承装配包络重叠约 6 mm，
仅显式排除此装配对，并保留 MuJoCo 默认父子体过滤。其他自碰撞和桌面碰撞保持启用。
M1 只证明演示轨迹没有桌面接触/明显穿透，不证明整个工作空间均无碰撞；
M2 还要校核夹指接触面、机械臂/容器碰撞和规划路径。

`base` 与官方 `base_link` 及本场景 world 重合，单位 m/rad；
应用层 `Pose` 四元数为 xyzw，MuJoCo 内部 wxyz 的换算留在适配器。
暂定 TCP 在 `link6` 坐标 `[0, 0, 0.125]` m，方向与 link6 相同，位于指尖夹持区域中间。
这不是已标定的官方控制器 TCP。原始指尖最高约 z=0.1358 m，本次 TCP 比尖端低
10.8 mm。全零关节 TCP 约 `[0.180667, −0.0000013, 0.224080]` m；
初始姿态约 `[0.287453, −0.0000018, 0.304709]` m。
独立 URDF 变换链在零位、初始姿态和另一组关节角核对全部 link 与 TCP 的位姿。

## 控制与接口边界

保留原有 `RobotPort`，增补可选 `JointRobotPort.move_joints()` 用于整臂调试。
`PiperMujocoRobot` 实现关节控制、夹爪、状态读取和保持当前位置的取消语义。
运动轨迹为五次平滑插值，目标速度不超过 0.5 rad/s、加速度不超过 1 rad/s²，
总夹爪开度速度不超过 0.04 m/s。这些均是本次仿真设置。

| 顺序 | J1 | J2 | J3 | J4 | J5 | J6 | J7 | J8 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 仿真 Kp | 180 | 220 | 180 | 50 | 40 | 25 | 600 | 600 |
| 仿真 Kd | 12 | 16 | 12 | 3 | 3 | 2 | 6 | 6 |
| 仿真努力上限 | 25 | 25 | 18 | 8 | 8 | 5 | 10 | 10 |

J1–J6 努力单位 N·m，J7/J8 为 N；增益分别按角度/位移及其速度使用相应单位。
PD 加 `qfrc_bias` 补偿后限幅，通过单位传动比 motor 执行，不使用官方示例的大力矩上限。
这些值不作为实机控制参数，bias 补偿依赖模型且会掩盖质量误差。

布尔结果兼容现有技能：`True` 表示命令执行并达到误差/速度阈值，`False` 表示拒绝或未到达；
详细状态在 `last_execution` 区分 `rejected/failed/succeeded/cancelled`。
角度最终误差阈值 0.01 rad，单指位移误差 0.5 mm，最终各自由度速度数值阈值 0.01。
`cancel()` 保持当前关节目标，不等同于真机急停。执行期穿透监测是反应式诊断，不是路径规划。
M1 的 `move_to(Pose)` 明确返回尚未实现 IK，防止上层误当抓放能力可用。

未来边界保持：自然语言 → CaP 任务程序 → 共用技能 → RobotPort →
PiPER MuJoCo / PiPER ROS 2；技能层不读写 MuJoCo。当前 state 数据明确标为
`mujoco_sim_state`，没有包装成相机结果。

## 本次验收与证据

- 38.514 秒连续动力学执行，视频 25 fps、963 帧，按真实仿真时间输出，无剪辑拼接。
- 18 个命令完成，六轴逐个正向运动/返回，指定组合姿态/返回，夹爪闭合/张开/半开/再张开。
- 基座固定；关节未越限；无桌面接触；无 MuJoCo 求解警告。
- 最大接触穿透约 `5.83e-8 m`，发生在闭爪两指接触；没有方块、容器抓放结论。
- 超关节限位、NaN、80 mm 开度命令均在仿真时间推进前拒绝。
- M1 专项测试 7 项通过。全仓回归 92 通过、1 失败：原有
  `test_physics_pick_place_from_second_start` 的简化漂浮夹爪未形成双侧接触；
  单独复现仍失败，该旧适配器未在本次修改，不作为 M1 的成功证据。
  回归首次定位到 `9a51151`，详见[旧夹爪回归记录](legacy-gripper-regression.md)。

## 补修规则

后续技能释放开度从 `gripper_opening_limits_m` 读取；PiPER 使用 0–70 mm，
Mock 设备使用自己的 0–80 mm 配置。没有合法设备配置时，技能不会把动作序列标为成功。
释放成功还必须由设备提供 `placement_verified=true`、来源/目标身份以及目标位置误差或
位置测量；只有 `held_object=null`、`holding=false` 或 `placement_verified=false`
均会保持 `uncertain`，仍持有来源物体也会拒绝成功。

`PiperMujocoRobot.cancel()` 使用线程安全取消请求；轨迹循环在每个物理步及保持阶段检查，
中止时冻结于已积分状态并保留 `last_execution.status=cancelled`。新的运动命令开始新一代轨迹。

`artifacts/piper_m1/result.json` 保存每条命令、起止时间、达标误差和拒绝原因；
`joint_trace.csv` 每个 1 ms 物理步记录关节目标/位置/速度/努力、TCP、开度和碰撞；
`model_audit.json` 保存来源、关节映射、质量惯量、TCP 和控制参数；
`adapted_model.xml` 保存实际运行的生成模型（meshdir 指向安装资源位置）。

## 未解决项与后续里程碑

1. 实物 PiPER 具体型号/硬件代际、固件版本、原装夹爪版本未确认；当前 URDF 为有条件选择。
2. 关节零位/方向、70/80 mm 夹爪行程、法兰到实际 TCP 的变换需现场核验。
3. 上游质量惯量疑点、凸包碰撞精度、夹爪内侧接触几何需 M2 前进一步校核。
4. M2：IK、可达性、避碰轨迹和完整物理抓放，记录成功及失败，并验证释放、撤离、稳定落入容器。
5. M3：实际模型 API 和受控 CaP 程序驱动同一 PiPER 技能流程；现有回退规则不作为验收。
6. M4：D435i 图像/深度观测、ROS 2 Humble 执行适配、相机外参、基座/桌面坐标、TCP、控制参数及现场验收。

后续尽量保留任务程序和技能逻辑，更换执行/观测适配器和现场配置；不承诺只切换一个参数即可真机成功。
此前漂浮夹爪实验仅作为独立接触测试保留，不作为项目主演示。
