import math
import unittest

from config import ANIMATION_STEP_CM
from models import Obstacle, Pose
from route_engine import RouteEngine, is_valid_pose, planned_step


class B2RouteTest(unittest.TestCase):
    def test_route_completes_all_targets_without_collisions(self):
        obstacles = [
            Obstacle(1, 60, 100, "N"),
            Obstacle(2, 130, 50, "W"),
            Obstacle(3, 150, 140, "S"),
            Obstacle(4, 80, 160, "E"),
            Obstacle(5, 40, 70, "N"),
        ]
        start = Pose(20, 20, math.pi / 2)
        engine = RouteEngine(start, obstacles)
        route_order = engine.route_order
        targets = engine.route_targets
        waypoints = engine.waypoints
        waypoint_targets = engine.waypoint_target_indices

        pose = start
        completed_targets = 0
        waypoint_index = 0
        route_index = 0
        max_steps = 1000
        steps = 0

        for _ in range(max_steps):
            steps += 1
            if waypoint_index >= len(waypoints):
                break
            waypoint = waypoints[waypoint_index]
            pose = planned_step(pose, waypoint, ANIMATION_STEP_CM)
            self.assertTrue(is_valid_pose(pose, obstacles))
            if pose.x == waypoint.x and pose.y == waypoint.y:
                target_index = waypoint_targets[waypoint_index]
                waypoint_index += 1
                target_segment_finished = (
                    waypoint_index >= len(waypoints)
                    or waypoint_targets[waypoint_index] != target_index
                )
                if target_segment_finished and target_index == route_index:
                    pose = targets[target_index]
                    completed_targets += 1
                    route_index += 1

        self.assertEqual(route_order, [5, 1, 4, 3, 2])
        self.assertEqual(completed_targets, 5)
        self.assertEqual(route_index, 5)
        self.assertEqual(waypoint_index, len(waypoints))
        self.assertLessEqual(steps, max_steps)


if __name__ == "__main__":
    unittest.main()