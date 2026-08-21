"""Hybrid-A* planner for a rectangular, non-holonomic robot."""

import heapq
import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

from config import (
    ARENA_CM,
    COLLISION_CHECK_STEP_CM,
    FORWARD_SPEED_CM_S,
    MIN_TURNING_RADIUS_CM,
    OBSTACLE_CM,
    ROBOT_LENGTH_CM,
    ROBOT_WIDTH_CM,
    REVERSE_PENALTY,
    REVERSE_SPEED_CM_S,
    SAFETY_MARGIN_CM,
    TURNING_SPEED_CM_S,
)
from models import Obstacle, Pose
from motion import normalise_angle
from route_engine import RouteEngine

HEADING_BINS = 32
# One full-steer primitive advances exactly one heading bin.  Cardinal target
# headings are therefore reachable without an in-place heading correction.
PRIMITIVE_CM = MIN_TURNING_RADIUS_CM * 2 * math.pi / HEADING_BINS
STEERS = (-1, 0, 1)
GEARS = (1, -1)


@dataclass(frozen=True)
class MotionSegment:
    """One executable lattice movement (gear plus steering direction)."""

    gear: int
    steer: int
    distance_cm: float

    @property
    def command(self) -> str:
        if self.steer < 0:
            turn = "R"
        elif self.steer > 0:
            turn = "L"
        else:
            turn = ""
        return ("F" if self.gear > 0 else "B") + turn

    @property
    def travel_time(self) -> float:
        speed = TURNING_SPEED_CM_S if self.steer else (
            FORWARD_SPEED_CM_S if self.gear > 0 else REVERSE_SPEED_CM_S
        )
        penalty = REVERSE_PENALTY if self.gear < 0 else 1.0
        return self.distance_cm / speed * penalty


@dataclass
class CarPlan:
    poses: List[Pose]
    segments: List[MotionSegment]

    @property
    def travel_time(self) -> float:
        return sum(segment.travel_time for segment in self.segments)


def _corners(pose: Pose) -> List[Tuple[float, float]]:
    half_l = ROBOT_LENGTH_CM / 2
    half_w = ROBOT_WIDTH_CM / 2
    cosine, sine = math.cos(pose.theta), math.sin(pose.theta)
    return [
        (pose.x + x * cosine - y * sine, pose.y + x * sine + y * cosine)
        for x, y in ((-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w))
    ]


def _overlap_on_axis(points: List[Tuple[float, float]], rect: Tuple[float, float, float, float], axis: Tuple[float, float]) -> bool:
    values = [x * axis[0] + y * axis[1] for x, y in points]
    left, right, bottom, top = rect
    other = [x * axis[0] + y * axis[1] for x, y in ((left, bottom), (right, bottom), (right, top), (left, top))]
    return max(values) > min(other) and max(other) > min(values)


def footprint_is_safe(pose: Pose, obstacles: Iterable[Obstacle]) -> bool:
    """Collision check the oriented robot rectangle, including safety margin."""
    points = _corners(pose)
    if any(x < 0 or x > ARENA_CM or y < 0 or y > ARENA_CM for x, y in points):
        return False
    robot_axes = ((math.cos(pose.theta), math.sin(pose.theta)), (-math.sin(pose.theta), math.cos(pose.theta)))
    for obstacle in obstacles:
        rect = (
            obstacle.x - SAFETY_MARGIN_CM,
            obstacle.x + OBSTACLE_CM + SAFETY_MARGIN_CM,
            obstacle.y - SAFETY_MARGIN_CM,
            obstacle.y + OBSTACLE_CM + SAFETY_MARGIN_CM,
        )
        if all(_overlap_on_axis(points, rect, axis) for axis in ((1, 0), (0, 1), *robot_axes)):
            return False
    return True


def apply_primitive(pose: Pose, gear: int, steer: int, distance: float = PRIMITIVE_CM) -> Pose:
    signed_distance = gear * distance
    if steer == 0:
        return Pose(
            pose.x + signed_distance * math.cos(pose.theta),
            pose.y + signed_distance * math.sin(pose.theta),
            pose.theta,
        )
    delta = signed_distance * steer / MIN_TURNING_RADIUS_CM
    theta = normalise_angle(pose.theta + delta)
    radius = MIN_TURNING_RADIUS_CM / steer
    return Pose(
        pose.x + radius * (math.sin(theta) - math.sin(pose.theta)),
        pose.y - radius * (math.cos(theta) - math.cos(pose.theta)),
        theta,
    )


def primitive_is_safe(start: Pose, gear: int, steer: int, obstacles: Iterable[Obstacle], distance: float = PRIMITIVE_CM) -> bool:
    samples = max(1, math.ceil(distance / COLLISION_CHECK_STEP_CM))
    return all(footprint_is_safe(apply_primitive(start, gear, steer, distance * i / samples), obstacles) for i in range(1, samples + 1))


def _key(pose: Pose) -> Tuple[int, int, int]:
    heading = int(round((normalise_angle(pose.theta) + math.pi) / (2 * math.pi) * HEADING_BINS)) % HEADING_BINS
    return round(pose.x / PRIMITIVE_CM), round(pose.y / PRIMITIVE_CM), heading


def _goal_error(pose: Pose, goal: Pose) -> Tuple[float, float]:
    return math.hypot(goal.x - pose.x, goal.y - pose.y), abs(normalise_angle(goal.theta - pose.theta))


def find_car_plan(start: Pose, goal: Pose, obstacles: Iterable[Obstacle], max_expansions: int = 80000) -> CarPlan:
    """Find forward/reverse straight and minimum-radius-arc poses."""
    obstacle_list = list(obstacles)
    if not footprint_is_safe(start, obstacle_list) or not footprint_is_safe(goal, obstacle_list):
        return CarPlan([], [])
    # Boundary viewing poses can only be occupied at their exact cardinal
    # heading. Plan to an aligned inset pose, then use a straight final motion.
    inset = ROBOT_LENGTH_CM / 2 + COLLISION_CHECK_STEP_CM
    search_goal = goal
    if goal.x <= inset:
        search_goal = Pose(goal.x + PRIMITIVE_CM, goal.y, goal.theta)
    elif goal.x >= ARENA_CM - inset:
        search_goal = Pose(goal.x - PRIMITIVE_CM, goal.y, goal.theta)
    elif goal.y <= inset:
        search_goal = Pose(goal.x, goal.y + PRIMITIVE_CM, goal.theta)
    elif goal.y >= ARENA_CM - inset:
        search_goal = Pose(goal.x, goal.y - PRIMITIVE_CM, goal.theta)

    start_key = _key(start)
    queue = [(0.0, 0.0, 0, start_key)]
    poses = {start_key: start}
    costs = {start_key: 0.0}
    parents: dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {start_key: None}
    actions: dict[Tuple[int, int, int], MotionSegment] = {}
    serial = 0
    found = None
    for _ in range(max_expansions):
        if not queue:
            break
        _, cost, _, key = heapq.heappop(queue)
        if cost != costs.get(key):
            continue
        pose = poses[key]
        distance_error, heading_error = _goal_error(pose, search_goal)
        if distance_error <= PRIMITIVE_CM * 3 and heading_error <= math.radians(8):
            found = key
            break
        for gear in GEARS:
            for steer in STEERS:
                candidate = apply_primitive(pose, gear, steer)
                candidate_key = _key(candidate)
                if candidate_key == key or not primitive_is_safe(pose, gear, steer, obstacle_list):
                    continue
                segment = MotionSegment(gear, steer, PRIMITIVE_CM)
                move_cost = segment.travel_time
                next_cost = cost + move_cost
                if next_cost >= costs.get(candidate_key, math.inf):
                    continue
                costs[candidate_key] = next_cost
                poses[candidate_key] = candidate
                parents[candidate_key] = key
                actions[candidate_key] = segment
                distance, angle = _goal_error(candidate, search_goal)
                heuristic = distance / max(FORWARD_SPEED_CM_S, REVERSE_SPEED_CM_S)
                heuristic += MIN_TURNING_RADIUS_CM * angle / TURNING_SPEED_CM_S
                serial += 1
                heapq.heappush(queue, (next_cost + 1.2 * heuristic, next_cost, serial, candidate_key))
    if found is None:
        return CarPlan([], [])
    result = []
    result_segments = []
    while found is not None:
        result.append(poses[found])
        if parents[found] is not None:
            result_segments.append(actions[found])
        found = parents[found]
    result.reverse()
    result_segments.reverse()
    # The exact target is a viewing-state marker. The preceding lattice pose is
    # within one primitive and the required heading tolerance.
    if search_goal != goal:
        result.append(search_goal)
        distance = math.hypot(goal.x - search_goal.x, goal.y - search_goal.y)
        direction = (goal.x - search_goal.x) * math.cos(goal.theta) + (goal.y - search_goal.y) * math.sin(goal.theta)
        result_segments.append(MotionSegment(1 if direction >= 0 else -1, 0, distance))
        samples = max(1, math.ceil(distance / COLLISION_CHECK_STEP_CM))
        result.extend(
            Pose(
                search_goal.x + (goal.x - search_goal.x) * index / samples,
                search_goal.y + (goal.y - search_goal.y) * index / samples,
                goal.theta,
            )
            for index in range(1, samples + 1)
        )
    else:
        result.append(goal)
        distance = math.hypot(goal.x - result[-2].x, goal.y - result[-2].y)
        if distance:
            direction = (goal.x - result[-2].x) * math.cos(goal.theta) + (goal.y - result[-2].y) * math.sin(goal.theta)
            result_segments.append(MotionSegment(1 if direction >= 0 else -1, 0, distance))
    return CarPlan(result, result_segments)


def find_car_path(start: Pose, goal: Pose, obstacles: Iterable[Obstacle], max_expansions: int = 80000) -> List[Pose]:
    """Compatibility wrapper returning only executable poses."""
    return find_car_plan(start, goal, obstacles, max_expansions).poses


@dataclass
class CarRouteEngine:
    """B.3 route and executable physical poses, retaining B.2 separately."""

    start_pose: Pose
    obstacles: List[Obstacle]
    b2: RouteEngine = field(init=False)
    route_order: List[int] = field(init=False)
    route_targets: List[Pose] = field(init=False)
    route_paths: List[List[Pose]] = field(init=False, default_factory=list)
    executable_path: List[Pose] = field(init=False, default_factory=list)
    executable_segments: List[MotionSegment] = field(init=False, default_factory=list)
    target_path_indices: List[int] = field(init=False, default_factory=list)
    estimated_travel_time: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        self.obstacles = list(self.obstacles)
        self.b2 = RouteEngine(self.start_pose, self.obstacles)
        targets = [self.b2.route_targets[self.b2.route_order.index(item.obstacle_id)] for item in self.obstacles]

        # Solve the small asymmetric travelling-salesperson problem using
        # speed-aware lattice transition costs. Reverse manoeuvres are part of
        # every transition search, so B.3 is not tied to B.2's distance order.
        transition_plans: dict[Tuple[int, int], CarPlan] = {}
        for destination, target in enumerate(targets):
            transition_plans[(-1, destination)] = find_car_plan(self.start_pose, target, self.obstacles)
        for source, start in enumerate(targets):
            for destination, target in enumerate(targets):
                if source != destination:
                    transition_plans[(source, destination)] = find_car_plan(start, target, self.obstacles)

        states: dict[Tuple[int, int], Tuple[float, Tuple[int, ...]]] = {}
        for destination in range(len(targets)):
            plan = transition_plans[(-1, destination)]
            if plan.poses:
                states[(1 << destination, destination)] = (plan.travel_time, (destination,))
        for mask_size in range(1, len(targets)):
            for (mask, last), (cost, order) in list(states.items()):
                if mask.bit_count() != mask_size:
                    continue
                for destination in range(len(targets)):
                    if mask & (1 << destination):
                        continue
                    plan = transition_plans[(last, destination)]
                    if not plan.poses:
                        continue
                    key = (mask | (1 << destination), destination)
                    candidate = (cost + plan.travel_time, order + (destination,))
                    if key not in states or candidate[0] < states[key][0]:
                        states[key] = candidate
        full_mask = (1 << len(targets)) - 1
        completed = [value for (mask, _), value in states.items() if mask == full_mask]
        if not completed:
            raise ValueError("No car-feasible B.3 route exists")
        self.estimated_travel_time, selected_order = min(completed, key=lambda item: item[0])
        self.route_targets = [targets[index] for index in selected_order]
        self.route_order = [self.obstacles[index].obstacle_id for index in selected_order]
        previous_index = -1
        for target_index in selected_order:
            plan = transition_plans[(previous_index, target_index)]
            path = plan.poses
            self.route_paths.append(path)
            self.executable_path.extend(path if not self.executable_path else path[1:])
            self.executable_segments.extend(plan.segments)
            self.target_path_indices.append(len(self.executable_path) - 1)
            previous_index = target_index
        self.reset()

    def reset(self) -> None:
        self.pose = Pose(self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        self.path_index = 0
        self.route_index = 0
        self.visited_ids: List[int] = []
        self.running = False
        self.active_command = "-"

    def toggle(self) -> None:
        if self.running:
            self.running = False
        else:
            if self.path_index >= len(self.executable_path) - 1:
                self.reset()
            self.running = True

    def step(self) -> Pose:
        if not self.running:
            return self.pose
        if self.path_index + 1 >= len(self.executable_path):
            self.running = False
            return self.pose
        self.path_index += 1
        self.pose = self.executable_path[self.path_index]
        if self.path_index - 1 < len(self.executable_segments):
            self.active_command = self.executable_segments[self.path_index - 1].command
        while self.route_index < len(self.target_path_indices) and self.path_index >= self.target_path_indices[self.route_index]:
            self.visited_ids.append(self.route_order[self.route_index])
            self.route_index += 1
        if self.path_index + 1 >= len(self.executable_path):
            self.running = False
        return self.pose

    @property
    def completed_targets(self) -> int:
        return self.route_index

    @property
    def finished(self) -> bool:
        return self.completed_targets == len(self.route_targets) and not self.running
