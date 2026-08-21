"""Collision-aware pathfinding for B.2 target routing."""

import heapq
import math
from typing import Iterable, List, Tuple

from config import ARENA_CM, OBSTACLE_CM, ROBOT_WIDTH_CM, SAFETY_MARGIN_CM
from models import Obstacle, Pose

GRID_SIZE_CM = 5
ROBOT_HALF_WIDTH_CM = ROBOT_WIDTH_CM / 2 + SAFETY_MARGIN_CM
ARENA_MIN_CM = ROBOT_HALF_WIDTH_CM
ARENA_MAX_CM = ARENA_CM - ROBOT_HALF_WIDTH_CM


def virtual_obstacle_bounds(obstacle: Obstacle) -> Tuple[float, float, float, float]:
    """Return the inflated 40x40 cm box around a 10x10 obstacle."""
    return (
        obstacle.x - ROBOT_HALF_WIDTH_CM,
        obstacle.x + OBSTACLE_CM + ROBOT_HALF_WIDTH_CM,
        obstacle.y - ROBOT_HALF_WIDTH_CM,
        obstacle.y + OBSTACLE_CM + ROBOT_HALF_WIDTH_CM,
    )


def point_in_virtual_obstacle(x: float, y: float, obstacles: Iterable[Obstacle]) -> bool:
    for obstacle in obstacles:
        min_x, max_x, min_y, max_y = virtual_obstacle_bounds(obstacle)
        if min_x < x < max_x and min_y < y < max_y:
            return True
    return False


def is_valid_point(x: float, y: float, obstacles: Iterable[Obstacle]) -> bool:
    if not (ARENA_MIN_CM <= x <= ARENA_MAX_CM and ARENA_MIN_CM <= y <= ARENA_MAX_CM):
        return False
    return not point_in_virtual_obstacle(x, y, obstacles)


def nearest_cell(value: float) -> int:
    return int(round(value / GRID_SIZE_CM))


def cell_center(cell_x: int, cell_y: int) -> Tuple[float, float]:
    return (cell_x * GRID_SIZE_CM + GRID_SIZE_CM / 2, cell_y * GRID_SIZE_CM + GRID_SIZE_CM / 2)


def valid_cell(cell_x: int, cell_y: int, obstacles: Iterable[Obstacle]) -> bool:
    if cell_x < 0 or cell_y < 0:
        return False
    x, y = cell_center(cell_x, cell_y)
    return is_valid_point(x, y, obstacles)


def segment_is_clear(start: Tuple[float, float], end: Tuple[float, float], obstacles: Iterable[Obstacle]) -> bool:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    samples = max(1, math.ceil(distance / GRID_SIZE_CM))
    for index in range(samples + 1):
        fraction = index / samples
        x = start[0] + fraction * (end[0] - start[0])
        y = start[1] + fraction * (end[1] - start[1])
        if not is_valid_point(x, y, obstacles):
            return False
    return True


def find_path(start_pose: Pose, target_pose: Pose, obstacles: Iterable[Obstacle]) -> List[Tuple[float, float]]:
    """Return a list of collision-free 5 cm grid waypoints from start to target."""
    obstacles = list(obstacles)
    start_x = start_pose.x
    start_y = start_pose.y
    target_x = target_pose.x
    target_y = target_pose.y

    if not is_valid_point(start_x, start_y, obstacles):
        return []
    if not is_valid_point(target_x, target_y, obstacles):
        return []

    nearest_start_cell = (nearest_cell(start_x), nearest_cell(start_y))
    candidate_start_cells = []
    for cell_x in range(max(0, nearest_start_cell[0] - 2), nearest_start_cell[0] + 3):
        for cell_y in range(max(0, nearest_start_cell[1] - 2), nearest_start_cell[1] + 3):
            if valid_cell(cell_x, cell_y, obstacles):
                cell_x_world, cell_y_world = cell_center(cell_x, cell_y)
                distance = math.hypot(cell_x_world - start_x, cell_y_world - start_y)
                candidate_start_cells.append((distance, cell_x, cell_y))
    if not candidate_start_cells:
        return []
    candidate_start_cells.sort()
    start_cell = (candidate_start_cells[0][1], candidate_start_cells[0][2])

    target_cell = None
    nearest_target_cell = (nearest_cell(target_x), nearest_cell(target_y))
    candidate_cells = []
    for cell_x in range(max(0, nearest_target_cell[0] - 2), nearest_target_cell[0] + 3):
        for cell_y in range(max(0, nearest_target_cell[1] - 2), nearest_target_cell[1] + 3):
            if valid_cell(cell_x, cell_y, obstacles):
                cell_x_world, cell_y_world = cell_center(cell_x, cell_y)
                distance = math.hypot(cell_x_world - target_x, cell_y_world - target_y)
                candidate_cells.append((distance, cell_x, cell_y))
    if not candidate_cells:
        return []
    candidate_cells.sort()
    target_cell = (candidate_cells[0][1], candidate_cells[0][2])

    open_set: List[Tuple[float, float, int, int]] = []
    heapq.heappush(open_set, (0.0, 0.0, start_cell[0], start_cell[1]))
    came_from: dict[Tuple[int, int], Tuple[int, int] | None] = {start_cell: None}
    g_score: dict[Tuple[int, int], float] = {start_cell: 0.0}
    f_score: dict[Tuple[int, int], float] = {start_cell: math.hypot(start_cell[0] - target_cell[0], start_cell[1] - target_cell[1])}

    while open_set:
        _, _, cell_x, cell_y = heapq.heappop(open_set)
        current = (cell_x, cell_y)
        if current == target_cell:
            break

        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            next_cell = (cell_x + dx, cell_y + dy)
            if next_cell[0] < 0 or next_cell[1] < 0:
                continue
            if next_cell[0] > 40 or next_cell[1] > 40:
                continue
            if not valid_cell(next_cell[0], next_cell[1], obstacles):
                continue
            if not segment_is_clear(cell_center(cell_x, cell_y), cell_center(next_cell[0], next_cell[1]), obstacles):
                continue

            step_cost = math.hypot(dx * GRID_SIZE_CM, dy * GRID_SIZE_CM)
            tentative_g = g_score[current] + step_cost
            if next_cell not in g_score or tentative_g < g_score[next_cell]:
                came_from[next_cell] = current
                g_score[next_cell] = tentative_g
                heuristic = math.hypot(next_cell[0] - target_cell[0], next_cell[1] - target_cell[1]) * GRID_SIZE_CM
                f_score[next_cell] = tentative_g + heuristic
                heapq.heappush(open_set, (f_score[next_cell], tentative_g, next_cell[0], next_cell[1]))

    if target_cell not in came_from and target_cell != start_cell:
        return []

    path_cells: List[Tuple[int, int]] = []
    current = target_cell
    while current is not None:
        path_cells.append(current)
        current = came_from[current]
    path_cells.reverse()

    waypoints: List[Tuple[float, float]] = [(start_x, start_y)]
    for cell_x, cell_y in path_cells[1:]:
        waypoints.append(cell_center(cell_x, cell_y))
    if waypoints[-1] != (target_x, target_y):
        if not segment_is_clear(waypoints[-1], (target_x, target_y), obstacles):
            return []
        waypoints.append((target_x, target_y))
    return waypoints
