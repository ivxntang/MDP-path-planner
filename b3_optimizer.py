"""Exhaustive, non-GUI shortest-time route optimizer for B.3."""

import itertools
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from config import FORWARD_SPEED_CM_S, HEADING_CHANGE_PENALTY_S
from models import Obstacle, Pose
from route_engine import find_safe_path, planned_step
from targets import viewing_target


def _angle_difference(first: float, second: float) -> float:
    return abs((second - first + math.pi) % (2 * math.pi) - math.pi)


def path_distance(path: Sequence[Pose]) -> float:
    """Return the distance along a waypoint path in centimetres."""
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(path, path[1:]))


def heading_change_count(path: Sequence[Pose]) -> int:
    """Count travel-direction changes, including alignment at both ends."""
    if len(path) < 2:
        return int(_angle_difference(path[0].theta, path[-1].theta) > 1e-9) if path else 0

    headings = [
        math.atan2(b.y - a.y, b.x - a.x)
        for a, b in zip(path, path[1:])
        if not (math.isclose(a.x, b.x) and math.isclose(a.y, b.y))
    ]
    if not headings:
        return int(_angle_difference(path[0].theta, path[-1].theta) > 1e-9)

    changes = int(_angle_difference(path[0].theta, headings[0]) > 1e-9)
    changes += sum(_angle_difference(a, b) > 1e-9 for a, b in zip(headings, headings[1:]))
    changes += int(_angle_difference(headings[-1], path[-1].theta) > 1e-9)
    return changes


def estimate_route_time(
    paths: Sequence[Sequence[Pose]],
    forward_speed_cm_s: float = FORWARD_SPEED_CM_S,
    heading_change_penalty_s: float = HEADING_CHANGE_PENALTY_S,
) -> float:
    """Estimate seconds from path distance and a fixed heading-change penalty."""
    if forward_speed_cm_s <= 0:
        raise ValueError("Forward speed must be positive")
    distance = sum(path_distance(path) for path in paths)
    changes = sum(heading_change_count(path) for path in paths)
    return distance / forward_speed_cm_s + changes * heading_change_penalty_s


@dataclass(frozen=True)
class B3Plan:
    order: Tuple[int, ...]
    targets: Tuple[Pose, ...]
    paths: Tuple[Tuple[Pose, ...], ...]
    total_distance: float
    heading_changes: int
    estimated_time: float
    orders_evaluated: int
    valid_orders: int

    @property
    def waypoints(self) -> Tuple[Pose, ...]:
        return tuple(waypoint for path in self.paths for waypoint in path[1:])


def optimize_route(start_pose: Pose, obstacles: Iterable[Obstacle]) -> B3Plan:
    """Evaluate all 120 target orders and return the quickest valid one."""
    obstacle_list = list(obstacles)
    if len(obstacle_list) != 5:
        raise ValueError("B.3 requires exactly five viewing targets")
    if len({obstacle.obstacle_id for obstacle in obstacle_list}) != 5:
        raise ValueError("B.3 obstacle IDs must be unique")

    best = None
    orders_evaluated = 0
    valid_orders = 0
    for order in itertools.permutations(obstacle_list):
        orders_evaluated += 1
        current = start_pose
        targets: List[Pose] = []
        paths: List[Tuple[Pose, ...]] = []
        valid = True
        for obstacle in order:
            target = viewing_target(obstacle)
            path = find_safe_path(current, target, obstacle_list)
            if not path:
                valid = False
                break
            targets.append(target)
            paths.append(tuple(path))
            current = target
        if not valid:
            continue

        valid_orders += 1
        distance = sum(path_distance(path) for path in paths)
        changes = sum(heading_change_count(path) for path in paths)
        estimated_time = distance / FORWARD_SPEED_CM_S + changes * HEADING_CHANGE_PENALTY_S
        candidate = (
            estimated_time,
            tuple(obstacle.obstacle_id for obstacle in order),
            tuple(targets),
            tuple(paths),
            distance,
            changes,
        )
        if best is None or candidate[:2] < best[:2]:
            best = candidate

    if best is None:
        raise ValueError("No collision-free B.3 route exists")
    return B3Plan(
        order=best[1],
        targets=best[2],
        paths=best[3],
        total_distance=best[4],
        heading_changes=best[5],
        estimated_time=best[0],
        orders_evaluated=orders_evaluated,
        valid_orders=valid_orders,
    )


class B3RouteEngine:
    """Playback state for an optimized B.3 plan, independent of tkinter."""

    def __init__(self, start_pose: Pose, obstacles: Iterable[Obstacle]) -> None:
        self.start_pose = start_pose
        self.obstacles = list(obstacles)
        self.plan = optimize_route(start_pose, self.obstacles)
        self.route_order = list(self.plan.order)
        self.route_targets = list(self.plan.targets)
        self.route_paths = [list(path) for path in self.plan.paths]
        self.waypoints: List[Pose] = []
        self.waypoint_target_indices: List[int] = []
        for target_index, path in enumerate(self.route_paths):
            self.waypoints.extend(path[1:])
            self.waypoint_target_indices.extend([target_index] * (len(path) - 1))
        self.reset()

    def reset(self) -> None:
        self.running = False
        self.paused = False
        self.waypoint_index = 0
        self.route_index = 0
        self.pose = Pose(self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        self.visited_ids: List[int] = []
        self.completed_targets = 0

    def toggle(self) -> None:
        if self.running:
            self.paused = not self.paused
            return
        if self.completed_targets == len(self.route_targets):
            self.reset()
        self.running = True
        self.paused = False

    def step(self) -> Pose:
        if not self.running or self.paused or self.waypoint_index >= len(self.waypoints):
            if self.waypoint_index >= len(self.waypoints):
                self.running = False
            return self.pose
        waypoint = self.waypoints[self.waypoint_index]
        self.pose = planned_step(self.pose, waypoint)
        if self.pose.x == waypoint.x and self.pose.y == waypoint.y:
            target_index = self.waypoint_target_indices[self.waypoint_index]
            self.waypoint_index += 1
            segment_finished = (
                self.waypoint_index == len(self.waypoints)
                or self.waypoint_target_indices[self.waypoint_index] != target_index
            )
            if segment_finished:
                target = self.route_targets[target_index]
                self.pose = Pose(target.x, target.y, target.theta)
                self.visited_ids.append(self.route_order[target_index])
                self.completed_targets += 1
                self.route_index += 1
        if self.waypoint_index >= len(self.waypoints):
            self.running = False
        return self.pose

    @property
    def finished(self) -> bool:
        return self.completed_targets == len(self.route_targets) and not self.running
