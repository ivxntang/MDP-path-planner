import math
import unittest

from models import Pose
from route_engine import planned_step


class B2SteeringTest(unittest.TestCase):
    def test_first_step_turns_clockwise_toward_first_target(self):
        start = Pose(20, 20, math.pi / 2)
        first_target = Pose(45, 100, -math.pi / 2)

        result = planned_step(start, first_target, 1.5)

        self.assertGreater(result.x, 20)
        self.assertGreater(result.y, 20)
        self.assertLess(result.theta, math.pi / 2)
        self.assertAlmostEqual(result.theta, math.atan2(80, 25))


if __name__ == "__main__":
    unittest.main()