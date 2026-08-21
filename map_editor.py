"""Testable map-editing rules used by the Tk canvas."""

from dataclasses import replace
from typing import Iterable, Optional

from config import ARENA_CM, EDITOR_SNAP_CM, OBSTACLE_CM, START_ZONE_CM
from models import Obstacle

IMAGE_SIDES = ("N", "E", "S", "W")


def snap_to_grid(value: float) -> float:
    return max(0.0, min(ARENA_CM - OBSTACLE_CM, round(value / EDITOR_SNAP_CM) * EDITOR_SNAP_CM))


def placement_is_valid(candidate: Obstacle, obstacles: Iterable[Obstacle]) -> bool:
    if candidate.x < 0 or candidate.y < 0:
        return False
    if candidate.x + OBSTACLE_CM > ARENA_CM or candidate.y + OBSTACLE_CM > ARENA_CM:
        return False
    # Obstacles may touch, but may not enter the 40 cm square start zone.
    if candidate.x < START_ZONE_CM and candidate.y < START_ZONE_CM:
        return False
    for other in obstacles:
        if other.obstacle_id == candidate.obstacle_id:
            continue
        separated = (
            candidate.x + OBSTACLE_CM <= other.x
            or other.x + OBSTACLE_CM <= candidate.x
            or candidate.y + OBSTACLE_CM <= other.y
            or other.y + OBSTACLE_CM <= candidate.y
        )
        if not separated:
            return False
    return True


class MapEditorModel:
    def __init__(self, obstacles: Iterable[Obstacle]) -> None:
        self.obstacles = list(obstacles)

    def obstacle_at(self, x: float, y: float) -> Optional[Obstacle]:
        return next(
            (item for item in reversed(self.obstacles) if item.x <= x <= item.x + OBSTACLE_CM and item.y <= y <= item.y + OBSTACLE_CM),
            None,
        )

    def move(self, obstacle_id: int, x: float, y: float) -> bool:
        index = next((i for i, item in enumerate(self.obstacles) if item.obstacle_id == obstacle_id), None)
        if index is None:
            return False
        candidate = replace(self.obstacles[index], x=snap_to_grid(x), y=snap_to_grid(y))
        if not placement_is_valid(candidate, self.obstacles):
            return False
        self.obstacles[index] = candidate
        return True

    def cycle_face(self, obstacle_id: int) -> bool:
        index = next((i for i, item in enumerate(self.obstacles) if item.obstacle_id == obstacle_id), None)
        if index is None:
            return False
        obstacle = self.obstacles[index]
        side = obstacle.image_side.upper()
        next_side = IMAGE_SIDES[(IMAGE_SIDES.index(side) + 1) % len(IMAGE_SIDES)]
        self.obstacles[index] = replace(obstacle, image_side=next_side)
        return True
