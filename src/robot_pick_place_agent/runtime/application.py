from robot_pick_place_agent.adapters.robots.mock import MockRobot
from robot_pick_place_agent.perception.mock_scene import MockSceneProvider
from robot_pick_place_agent.skills.pick_place import pick_and_place
from robot_pick_place_agent.core.models import TaskIntent


class Application:
    def __init__(self, robot=None, scene_provider=None):
        self.robot = robot or MockRobot()
        self.scene_provider = scene_provider or MockSceneProvider()

    def observe(self):
        return self.scene_provider.observe()

    def run(self, instruction: str):
        scene = self.observe()
        intent = TaskIntent(instruction, "red-block", "blue-box", scene.scene_id)
        return pick_and_place(self.robot, scene, intent)

    def simulate(self, instruction: str, start=(0.30, 0.05), xml_path=None, observation_mode="state"):
        """Run the same high-level instruction through the optional physics adapter."""
        if "红" not in instruction and "方块" not in instruction:
            raise ValueError("当前仿真场景只支持红色方块")
        if "蓝" not in instruction and "盒" not in instruction:
            raise ValueError("当前仿真场景只支持蓝色盒子")
        from robot_pick_place_agent.adapters.simulation.mujoco import run_physics_pick_place
        if xml_path:
            raise NotImplementedError("外部 PiPER MJCF 需要映射到当前适配器的 body/joint 名称；请先用 MujocoRobot(xml_path=...) 接入对应模型")
        return run_physics_pick_place(start, observation_mode=observation_mode)
