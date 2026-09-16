import time
from robot_pick_place_agent.core.models import Pose, SceneObject, SceneSnapshot


class MockSceneProvider:
    def observe(self) -> SceneSnapshot:
        return SceneSnapshot(
            scene_id="mock-scene-1", observed_at=time.time(),
            objects=(SceneObject("red-block", "方块", "红色", Pose("base", .30, .05, .03)),),
            targets=(SceneObject("blue-box", "盒子", "蓝色", Pose("base", .50, .05, .02), .08),),
        )
