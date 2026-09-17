# robot-pick-place-agent：仓库结构设计

本文定义项目的目标结构与接口边界，下列目录中部分仍是演进目标。
当前已有 CLI 和完整 PiPER MuJoCo M1 后端，状态见 [M1 报告](piper-m1.md)；真机控制尚未接入。
里程碑顺序为 M1 整臂关节控制 → M2 state 物理抓放 → M3 实际模型 API＋CaP → M4 相机与真机适配。

## 1. 目标和边界

首个部署目标：单台 PiPER、有夹爪、D435i、Ubuntu 22.04、ROS 2 Humble、单张 RTX 5080；CLI 输入自然语言后完成指定物体到指定容器/区域的抓取放置。

复用目标：更换语言模型、检测模型、RGB-D 相机、机械臂或实验桌面时，保持抓放任务和上层 CLI 的主要代码不变。

采用一个仓库、一个主要 Python 包和必要的 ROS 2 桥接包。暂不建立多个独立发布包、通用插件注册系统或分布式任务平台。对外按实际能力组合，不能把接口存在等同于支持任意机器人或任意物体。

## 2. 推荐目录

```text
robot-pick-place-agent/
├── README.md                         # 快速开始、当前实现状态、硬件要求
├── pyproject.toml                    # Python 包、CLI 入口、可选依赖
├── .gitignore
├── .env.example                      # 变量名占位，不含凭据
├── UPSTREAM.md                       # 上游、固定版本、许可证、复用范围
├── docs/
│   ├── architecture.md               # 层次、依赖方向、数据流
│   ├── interfaces.md                 # 接口、单位、坐标系、错误约定
│   ├── setup-piper-humble.md          # 首台机器人环境建立
│   ├── calibration.md                # 相机外参、TCP、验证方法
│   └── acceptance.md                 # 已有能力、验证步骤与结果
├── src/
│   └── robot_pick_place_agent/
│       ├── core/                     # 设备无关的数据和规则
│       │   ├── models.py             # 任务、观测、位姿、动作结果
│       │   ├── ports.py              # Python Protocol 接口声明
│       │   ├── geometry.py           # 坐标转换、区域关系、几何验证
│       │   └── errors.py             # 结构化错误及可恢复性
│       ├── perception/               # 检测→深度→基座坐标→场景快照
│       │   ├── scene_builder.py
│       │   ├── localization.py
│       │   └── association.py        # 执行前后对象对应关系
│       ├── skills/                   # 设备无关的任务流程
│       │   ├── pick_place.py         # 抓取、持物、放置阶段控制
│       │   ├── grasp_templates.py    # 明确适用范围的抓取几何模板
│       │   └── verification.py       # 真正的放置结果检查
│       ├── agent/                    # 语言理解、工具选择、反馈
│       │   ├── planner.py
│       │   ├── tools.py              # 对模型开放的有限高层工具
│       │   └── prompts/              # 任务说明和少量例子
│       ├── adapters/                 # 所有供应商/设备依赖
│       │   ├── robots/
│       │   │   ├── piper_ros2.py      # PiPER ROS 桥的客户端
│       │   │   └── mock.py            # 模拟反馈，不触达真机
│       │   ├── cameras/
│       │   │   ├── ros_rgbd.py        # 已对齐 RGB-D 数据接口
│       │   │   └── recorded.py        # 离线录制帧
│       │   ├── detectors/
│       │   │   ├── grounding_dino.py
│       │   │   └── color_blocks.py    # 受控方块场景的基线
│       │   └── llms/
│       │       └── api.py             # 一个已验证的 API 客户端起步
│       ├── runtime/                  # 组装模块、任务调度与运行记录
│       │   ├── application.py
│       │   ├── config.py
│       │   ├── task_runner.py        # 单臂单任务、取消、超时、去重
│       │   └── recorder.py
│       └── cli/                      # 命令入口和终端输出
│           └── main.py
├── ros2_ws/
│   ├── upstream.repos               # 第三方 ROS 源码及固定提交
│   └── src/
│       ├── robot_task_interfaces/    # 确有需要时定义的消息/action
│       └── piper_task_bridge/        # 驱动/MoveIt 的实际适配
├── configs/
│   ├── defaults.yaml
│   ├── robots/piper.example.yaml
│   ├── cameras/d435i.example.yaml
│   └── sites/tabletop.example.yaml   # 演示模板，不能直接当实机标定
├── examples/
│   ├── mock_pick_place/             # 无硬件、无 API Key 的入口
│   └── rgbd_replay/                 # 经许可的最小离线样例
├── tests/
│   ├── unit/                       # 坐标、状态、参数等纯逻辑
│   ├── contract/                   # 各适配器遵守相同接口
│   ├── integration/                # 回放、mock、桥接协议
│   └── hardware/                   # 明确显式开启的现场验证
├── scripts/                        # 环境检查、数据采样、启动辅助
└── .github/workflows/ci.yml         # 无硬件 CPU 检查，后续加入
```

目录是演进目标。初始化只建立有内容的文件；实现到某个模块时再创建对应目录，不批量堆空文件或空类。

大型模型、原始实验数据、实际标定、设备序列号、API 密钥和 ROS 构建产物不进入公开仓库。运行记录保存在用户指定的数据目录；配置模板只含可公开的字段和说明。

## 3. 核心复用规则

### 3.1 依赖朝核心汇聚

```text
CLI → runtime → agent / perception / skills → core
                         ↑
                 runtime 注入 adapters
                         ↓
                实际模型 / 相机 / ROS 桥
```

- `core` 不导入 ROS、PiPER SDK、Torch 或任何模型 API SDK。
- `skills` 只依赖核心接口，不写 PiPER 话题名、相机参数或模型名。
- `agent` 只读场景快照、提出合法工具调用，不直接发送关节命令。
- `agent.planner` 可采用 Code as Policies 风格的任务分解，但模型只能返回经过校验的有限工具调用；不执行生成的 Python、关节命令或任意模拟器代码。
- `adapters` 负责第三方数据到核心数据的转换，包含单位、坐标系和异常映射。
- `runtime` 根据配置显式组装实例，第一版不用动态扫描插件或自动注册。
- `cli` 只处理参数和展示，不复制抓放业务逻辑。

这使 mock、回放和真实设备可以接入同一条任务流程，同时保持来源和能力边界可见。

### 3.2 先定五个接口

以下是拟定合同，不是已实现函数签名：

| 接口 | 输入/输出职责 | 替换时影响范围 |
|---|---|---|
| `CameraPort` | 获取包含配套图像、深度、内参、时间戳和 frame 的 `FrameBundle` | 更换相机适配器和相机配置 |
| `DetectorPort` | 图像 + 目标查询 → `Detection` 列表，掩码可选 | 更换检测器及其配置 |
| `RobotPort` | 状态读取、动作规划/执行、夹爪控制、任务取消 | 更换机器人适配器与对应控制后端 |
| `TaskPlannerPort` | 用户指令 + 场景 + 可用工具 → 结构化任务意图或澄清 | 更换语言模型适配器 |
| `RunRecorderPort` | 接收结构化事件及数据引用 | 更换日志/数据存储方式 |

机器人接口需要描述支持的能力，例如末端运动、夹爪开口控制、可用反馈和已验证停止语义。规划失败、执行失败、执行中与实际完成不能统一成一个布尔值。

`pick_and_place` 是技能层功能，不放进相机或 LLM 适配器。运动的逆解、轨迹和具体控制器由机器人后端实现，技能层只负责任务阶段及其条件。

### 3.3 数据必须带上下文

| 数据 | 必须携带的关键内容 |
|---|---|
| `Pose` | frame_id、位置、旋转及其约定 |
| `FrameBundle` | RGB/深度配对、内参、深度单位、时间戳、相机 frame |
| `SceneSnapshot` | scene_id、观测时间、对象、目标、校准版本 |
| `SceneObject` | 快照内 ID、名称/属性、几何、定位质量、数据来源 |
| `TaskIntent` | 指令、选定对象/目标、引用场景、是否需要澄清 |
| `ActionResult` | task_id、实际阶段、成功/失败/不确定、错误和证据 |

统一使用米、弧度；固定四元数顺序。不同驱动的换算留在适配层。RGB-D 对齐后的内参与 frame 要配套，不能仅保存一个没有来源的 XYZ。

对象 ID 默认只在当前场景快照内有效。重新观察后需要关联对象或重新解析指令，不能依赖检测输出的数组顺序。

## 4. 现场配置独立于代码

把可复用算法、设备参数和现场标定区分开：

- 设备模板：机械臂型号、反馈/控制接口、相机模式、支持能力。
- 现场文件：固件与模型版本、TCP、相机外参、桌面/容器几何、工作区域、已验证准备姿态。
- 任务参数：物体目录、抓取模板、成功判据、超时和重试限制。
- 凭据：环境变量，不能混进可提交的配置。

第一版保留少量明确配置，不提前建立复杂的配置继承体系。启动时检查必需字段，示例配置不能悄悄使用虚构坐标启动真机。

## 5. ROS 与 GPU 环境的边界

ROS 进程负责设备、MoveIt 和实际反馈；Agent/视觉进程负责模型推理与任务调用。两者可以使用不同 Python 环境，避免一个环境同时背负全部依赖。

初始桥接选择一个本机通信合同即可，不同时实现 HTTP、gRPC、ROS 三套业务接口。建议 ROS 侧桥接向应用暴露本机任务接口及图像帧读取接口；绑定本机地址，执行任务串行化。ROS 内部长动作使用合适的 action/状态反馈；如果需要 C++ MoveIt 接口，可将其留在桥接包中，不影响 Python 应用。

所有任务请求带 task_id：网络超时后查询已有任务，不能因重发请求重复抓放。真实控制只允许一条发送路径，模拟关节状态不能接到实机控制话题。

GPU 可选依赖只由视觉适配器加载；CPU 环境导入核心包和运行 mock 时，不应因找不到 Torch、CUDA、ROS 或 API Key 而失败。

## 6. CLI 与测试入口

拟定 CLI：

```text
robot-agent doctor --config <site-config>
robot-agent observe --config <site-config>
robot-agent plan "把红色方块放进蓝色盒子" --config <site-config>
robot-agent run "把红色方块放进蓝色盒子" --config <site-config>
robot-agent status <task-id>
```

默认示例使用 mock，不连接 CAN。真实运行要求显式选择实机配置，缺少标定或能力不匹配时返回原因。`plan` 不执行运动，`run` 才提交任务。

首批值得写的测试：坐标变换方向/单位、目标 ID 与场景版本校验、观测过期、抓取失败后不执行放置、持物不确定时不盲目重抓、同一任务去重。CI 只运行无硬件测试；真机测试显式启用并留记录。

mock 用于验证逻辑和接口，不能替代物理仿真或真实抓取成功率。回放用于验证感知与定位，不能单独证明机械臂运动可行。

## 7. 上游复用与初始落地顺序

- PiPER ROS：外部依赖固定 humble 提交；必要改动在自己的桥接层，避免把上游源码散落到核心包。
- RealSense ROS：作为设备依赖；应用消费通用 RGB-D 合同。
- GroundingDINO：通过已验证的 Transformers 版本使用；权重缓存放仓库外。
- CaP：复用/改编提示和 API 组织，实际复制代码时记录来源与许可；模型客户端替换为当前 API。结构化工具调用路线不必保留原始任意代码执行。

项目开源许可证待定。复用上游代码时保留来源和许可说明，在发布可复用包前明确项目许可证。

建议初始化到以下程度后再扩展：

1. 远程公开仓库与简短 README。
2. 架构文档、接口合同、配置样例和忽略规则。
3. 核心数据类型、mock 机器人、假场景与一个可运行 CLI。
4. PiPER / ROS 桥和固定坐标抓放。
5. D435i、标定、检测和视觉抓放。
6. 模型 API、结果检查和真实验收。

每次增加一个适配器时，用相同合同验证其行为。复用性优先体现为“换组件时其他层不用改”，而不是目录数量或抽象层数量。
