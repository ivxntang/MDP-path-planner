"""Convert the image side of an obstacle into a camera viewing pose."""

import math

from config import OBSTACLE_CM, TARGET_STANDOFF_DISTANCE_CM
from models import Obstacle, Pose


def viewing_target(obstacle: Obstacle) -> Pose:
    """Return a pose outside the obstacle, facing its indicated image side.

    The target is deliberately placed at the obstacle's side centre.  The exact
    viewing distance can later be calibrated against the actual camera.
    """

    centre_x = obstacle.x + OBSTACLE_CM / 2
    centre_y = obstacle.y + OBSTACLE_CM / 2
    distance = OBSTACLE_CM / 2 + TARGET_STANDOFF_DISTANCE_CM
    side = obstacle.image_side.upper()

    if side == "N":
        return Pose(centre_x, centre_y + distance, -math.pi / 2)
    if side == "S":
        return Pose(centre_x, centre_y - distance, math.pi / 2)
    if side == "E":
        return Pose(centre_x + distance, centre_y, math.pi)
    if side == "W":
        return Pose(centre_x - distance, centre_y, 0)
    raise ValueError(f"Obstacle {obstacle.obstacle_id} has invalid image side: {side}")
