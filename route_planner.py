"""Nearest-neighbour route planner for B.2 target visitation."""

import math
from typing import Iterable, List, Tuple

from models import Obstacle, Pose
from targets import viewing_target


def euclidean_distance(a: Pose, b: Pose) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def plan_route(start_pose: Pose, obstacles: Iterable[Obstacle]) -> Tuple[List[int], List[Pose], float]:
    """Return the obstacle IDs in nearest-neighbour order, their target poses, and total distance."""

    remaining = list(obstacles)
    current_pose = start_pose
    visit_order: List[int] = []
    visit_targets: List[Pose] = []
    total_distance = 0.0

    while remaining:
        best_obstacle = None
        best_target = None
        best_distance = math.inf

        for obstacle in remaining:
            target = viewing_target(obstacle)
            distance = euclidean_distance(current_pose, target)
            if distance < best_distance:
                best_distance = distance
                best_obstacle = obstacle
                best_target = target

        if best_obstacle is None or best_target is None:
            break

        visit_order.append(best_obstacle.obstacle_id)
        visit_targets.append(best_target)
        total_distance += best_distance
        current_pose = best_target
        remaining.remove(best_obstacle)

    return visit_order, visit_targets, total_distance
