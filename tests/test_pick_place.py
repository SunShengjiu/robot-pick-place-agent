from robot_pick_place_agent.runtime.application import Application
from robot_pick_place_agent.core.models import ActionStatus


def test_mock_run_completes():
    result = Application().run("把红色方块放进蓝色盒子")
    assert result.status is ActionStatus.SUCCEEDED
