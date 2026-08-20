import math

from config import ANIMATION_STEP_CM
from models import Obstacle, Pose
from route_engine import (
    RouteEngine,
    find_safe_path,
    is_valid_pose,
    point_in_virtual_obstacle,
    virtual_obstacle_bounds,
)
from targets import viewing_target


SAMPLE_OBSTACLES = [
    Obstacle(1, 60, 100, "N"),
    Obstacle(2, 130, 50, "W"),
    Obstacle(3, 150, 140, "S"),
    Obstacle(4, 80, 160, "E"),
    Obstacle(5, 40, 70, "N"),
]
SECOND_LAYOUT = [
    Obstacle(1, 55, 125, "S"),
    Obstacle(2, 125, 45, "E"),
    Obstacle(3, 145, 125, "W"),
    Obstacle(4, 75, 155, "N"),
    Obstacle(5, 45, 65, "E"),
]


def run_to_completion(obstacles):
    engine = RouteEngine(Pose(20, 20, math.pi / 2), obstacles)
    engine.toggle()
    steps = 0
    while engine.running and steps < 10_000:
        engine.step()
        steps += 1
    return engine, steps


def test_five_targets_and_unique_route():
    engine = RouteEngine(Pose(20, 20, math.pi / 2), SAMPLE_OBSTACLES)
    assert len(engine.route_targets) == 5
    assert len(engine.route_order) == 5
    assert set(engine.route_order) == {1, 2, 3, 4, 5}
    assert len(set(engine.route_order)) == 5


def test_each_path_is_safe_and_target_ends_exactly():
    engine = RouteEngine(Pose(20, 20, math.pi / 2), SAMPLE_OBSTACLES)
    current = engine.start_pose
    for target, path in zip(engine.route_targets, engine.route_paths):
        assert path[0].x == current.x and path[0].y == current.y
        assert path[-1] == target
        for waypoint in path:
            assert is_valid_pose(waypoint, SAMPLE_OBSTACLES)
            assert not point_in_virtual_obstacle(waypoint.x, waypoint.y, SAMPLE_OBSTACLES)
        current = target


def test_first_target_is_reached():
    engine = RouteEngine(Pose(20, 20, math.pi / 2), SAMPLE_OBSTACLES)
    engine.toggle()
    while engine.completed_targets == 0:
        engine.step()
    assert engine.visited_ids[0] == engine.route_order[0]
    assert engine.completed_targets == 1
    assert engine.pose == engine.route_targets[0]


def test_end_to_end_reaches_all_targets_once_within_limit():
    engine, steps = run_to_completion(SAMPLE_OBSTACLES)
    assert engine.finished
    assert steps < 10_000
    assert engine.completed_targets == 5
    assert engine.visited_ids == engine.route_order
    assert engine.pose == engine.route_targets[-1]


def test_reset_clears_completion_state():
    engine, _ = run_to_completion(SAMPLE_OBSTACLES)
    engine.reset()
    assert engine.completed_targets == 0
    assert engine.visited_ids == []
    assert engine.route_index == 0
    assert engine.pose == engine.start_pose
    assert not engine.running


def test_pause_and_resume_preserve_route_state():
    engine = RouteEngine(Pose(20, 20, math.pi / 2), SAMPLE_OBSTACLES)
    engine.toggle()
    for _ in range(10):
        engine.step()
    before = engine.pose
    engine.toggle()
    assert engine.paused
    engine.step()
    assert engine.pose == before
    engine.toggle()
    assert not engine.paused
    engine.step()
    assert engine.pose != before


def test_second_valid_layout_completes_all_targets():
    engine, steps = run_to_completion(SECOND_LAYOUT)
    assert engine.finished
    assert steps < 10_000
    assert engine.completed_targets == 5
    assert engine.visited_ids == engine.route_order


def test_virtual_obstacle_dimensions_and_boundaries():
    obstacle = SAMPLE_OBSTACLES[0]
    assert virtual_obstacle_bounds(obstacle) == (45.0, 85.0, 85.0, 125.0)
    engine = RouteEngine(Pose(20, 20, math.pi / 2), SAMPLE_OBSTACLES)
    for waypoint in engine.waypoints:
        assert 15 <= waypoint.x <= 185
        assert 15 <= waypoint.y <= 185
