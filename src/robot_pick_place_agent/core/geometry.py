import math
from .models import Pose


def distance(a: Pose, b: Pose) -> float:
    if a.frame_id != b.frame_id:
        raise ValueError("poses must use the same frame")
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
