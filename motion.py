"""Simple kinematic movement for the visual B.1 simulator.

This is intentionally not the full Dubins planner yet.  It makes the robot
move like a car: straight movement and curves with a minimum turning radius.
"""

import math

from config import TURN_RADIUS_CM
from models import Pose


def normalise_angle(angle: float) -> float:
    """Keep an angle within [-pi, pi)."""

    return (angle + math.pi) % (2 * math.pi) - math.pi


def move_straight(pose: Pose, distance_cm: float) -> Pose:
    return Pose(
        pose.x + distance_cm * math.cos(pose.theta),
        pose.y + distance_cm * math.sin(pose.theta),
        pose.theta,
    )


def move_curve(pose: Pose, turn_angle_radians: float) -> Pose:
    """Move on a minimum-radius arc.

    Positive angles turn left; negative angles turn right.  This is the same
    car-like primitive used later when generating Dubins path segments.
    """

    radius = TURN_RADIUS_CM
    old_heading = pose.theta
    new_heading = normalise_angle(old_heading + turn_angle_radians)
    x = pose.x + radius * (math.sin(new_heading) - math.sin(old_heading))
    y = pose.y - radius * (math.cos(new_heading) - math.cos(old_heading))
    return Pose(x, y, new_heading)
