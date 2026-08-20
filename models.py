"""Small data structures shared by the simulator and later planner."""

from dataclasses import dataclass


@dataclass
class Pose:
    """Robot centre location in cm and its heading in radians.

    Heading convention: East = 0, North = pi/2, West = pi, South = -pi/2.
    """

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class Obstacle:
    """A 10 cm square obstacle whose image sits on one indicated face."""

    obstacle_id: int
    x: float  # bottom-left x coordinate, in cm
    y: float  # bottom-left y coordinate, in cm
    image_side: str  # N, E, S or W
