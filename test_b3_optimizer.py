import itertools
import math
import unittest

from b3_optimizer import B3RouteEngine, estimate_route_time, optimize_route
from models import Obstacle, Pose
from route_engine import find_safe_path, is_valid_pose
from targets import viewing_target


OBSTACLES = [
    Obstacle(1, 60, 100, "N"),
    Obstacle(2, 130, 50, "W"),
    Obstacle(3, 150, 140, "S"),
    Obstacle(4, 80, 160, "E"),
    Obstacle(5, 40, 70, "N"),
]
START = Pose(20, 20, math.pi / 2)


class B3OptimizerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = optimize_route(START, OBSTACLES)

    def test_visits_all_five_targets_exactly_once(self):
        self.assertEqual(self.plan.orders_evaluated, 120)
        self.assertEqual(len(self.plan.order), 5)
        self.assertEqual(set(self.plan.order), {1, 2, 3, 4, 5})

        engine = B3RouteEngine(START, OBSTACLES)
        engine.toggle()
        for _ in range(10_000):
            if not engine.running:
                break
            engine.step()
        self.assertTrue(engine.finished)
        self.assertEqual(engine.visited_ids, list(self.plan.order))
        self.assertEqual(len(set(engine.visited_ids)), 5)

    def test_every_b3_waypoint_is_safe(self):
        for path in self.plan.paths:
            for waypoint in path:
                self.assertTrue(is_valid_pose(waypoint, OBSTACLES))

    def test_selected_route_is_no_slower_than_every_valid_permutation(self):
        valid_times = []
        for order in itertools.permutations(OBSTACLES):
            current = START
            paths = []
            for obstacle in order:
                target = viewing_target(obstacle)
                path = find_safe_path(current, target, OBSTACLES)
                if not path:
                    break
                paths.append(path)
                current = target
            else:
                valid_times.append(estimate_route_time(paths))

        self.assertTrue(valid_times)
        self.assertLessEqual(self.plan.estimated_time, min(valid_times) + 1e-9)


if __name__ == "__main__":
    unittest.main()
