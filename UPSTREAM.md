# 上游来源

## PiPER M1

- 仓库：https://github.com/agilexrobotics/piper_ros
- 分支：`humble`
- 固定提交：`017ffefa64511bc6325bd77ddc4e16065c152051`
- 模型入口：[官方 MJCF](https://github.com/agilexrobotics/piper_ros/blob/017ffefa64511bc6325bd77ddc4e16065c152051/src/piper_description/mujoco_model/piper_description.xml)
- 结构、零位、质量惯量与限位依据：[当前带夹爪 URDF](https://github.com/agilexrobotics/piper_ros/blob/017ffefa64511bc6325bd77ddc4e16065c152051/src/piper_description/urdf/piper_description.urdf)
- 资源：`src/robot_pick_place_agent/assets/piper/`，包括 10 个原始 STL、原始 MJCF、当前/旧版 URDF、README、package.xml、LICENSE。
- 文件 SHA-256 与原路径：同目录 `source.json`；测试逐文件校验。
- 上游根许可证 MIT（Copyright 2024 RosenYin），原文随资源保留；上游 description/package.xml 仍为 TODO 许可证声明，此差异已记录。

没有修改上游资源字节。`piper_model.py` 在加载时使用 XML 结构化 API 生成适配模型；
生成结果保存在运行输出的 `adapted_model.xml`，具体变更见 [M1 核对报告](docs/piper-m1.md)。
官方仿真脚本使用旧 `mujoco_py`，未复制其控制实现；本项目使用原生 `mujoco` Python 包。

重新导入固定版本资源：

```bash
git clone --branch humble https://github.com/agilexrobotics/piper_ros.git /tmp/piper_ros
git -C /tmp/piper_ros checkout 017ffefa64511bc6325bd77ddc4e16065c152051
python3 tools/vendor_piper.py /tmp/piper_ros
```

## 其他计划接入

CaP 实际模型 API、ROS 2 PiPER 执行适配器、D435i 相机观测均不属于 M1 已完成功能。
后续按实际引入的版本追加来源记录；现有关键词回退和 mock 流程不作为 CaP 验收证据。
