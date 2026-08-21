import math
import unittest

from config import FORWARD_SPEED_CM_S, MIN_TURNING_RADIUS_CM, REVERSE_PENALTY, REVERSE_SPEED_CM_S, ROBOT_WIDTH_CM, SAFETY_MARGIN_CM
from map_editor import MapEditorModel, placement_is_valid
from models import Obstacle, Pose
from motion import normalise_angle
from physical_planner import CarRouteEngine, MotionSegment, apply_primitive, footprint_is_safe, primitive_is_safe
from targets import viewing_target


LAYOUTS = [
    [
        Obstacle(1, 60, 100, "N"), Obstacle(2, 130, 50, "W"),
        Obstacle(3, 150, 140, "S"), Obstacle(4, 80, 160, "E"),
        Obstacle(5, 40, 70, "N"),
    ],
    [
        Obstacle(1, 65, 100, "N"), Obstacle(2, 135, 50, "W"),
        Obstacle(3, 150, 135, "S"), Obstacle(4, 85, 160, "E"),
        Obstacle(5, 45, 70, "N"),
    ],
]


class MapEditorTests(unittest.TestCase):
    def test_drag_snaps_and_rejects_invalid_placements(self):
        editor = MapEditorModel([Obstacle(1, 60, 100, "N"), Obstacle(2, 80, 100, "E")])
        self.assertTrue(editor.move(1, 67, 113))
        self.assertEqual((editor.obstacles[0].x, editor.obstacles[0].y), (65, 115))
        self.assertFalse(editor.move(1, 82, 102))
        self.assertFalse(editor.move(1, 10, 10))

    def test_face_cycle_recalculates_target(self):
        editor = MapEditorModel([Obstacle(1, 60, 100, "N")])
        before = viewing_target(editor.obstacles[0])
        editor.cycle_face(1)
        after = viewing_target(editor.obstacles[0])
        self.assertEqual(editor.obstacles[0].image_side, "E")
        self.assertNotEqual(before, after)

    def test_placement_safety_rules(self):
        obstacles = [Obstacle(1, 50, 50, "N")]
        self.assertFalse(placement_is_valid(Obstacle(2, 35, 35, "N"), obstacles))
        self.assertFalse(placement_is_valid(Obstacle(2, 55, 55, "N"), obstacles))
        self.assertFalse(placement_is_valid(Obstacle(2, 195, 100, "N"), obstacles))


class PhysicalPlannerTests(unittest.TestCase):
    def test_configured_clearance_and_footprint_collision(self):
        obstacle = Obstacle(1, 60, 100, "N")
        half = ROBOT_WIDTH_CM / 2 + SAFETY_MARGIN_CM
        self.assertFalse(footprint_is_safe(Pose(60 - half + 0.1, 105, 0), [obstacle]))
        self.assertTrue(footprint_is_safe(Pose(60 - half, 105, 0), [obstacle]))

    def test_arc_obeys_minimum_turning_radius_and_checks_samples(self):
        start = Pose(100, 100, 0)
        end = apply_primitive(start, 1, 1)
        angle = abs(normalise_angle(end.theta - start.theta))
        chord = math.hypot(end.x - start.x, end.y - start.y)
        measured_radius = chord / (2 * math.sin(angle / 2))
        self.assertAlmostEqual(measured_radius, MIN_TURNING_RADIUS_CM, places=6)
        self.assertTrue(primitive_is_safe(start, 1, 1, []))
        for gear in (-1, 1):
            for steer in (-1, 0, 1):
                self.assertTrue(primitive_is_safe(start, gear, steer, []))

    def test_reverse_has_configured_speed_and_small_penalty(self):
        forward = MotionSegment(1, 0, 10).travel_time
        reverse = MotionSegment(-1, 0, 10).travel_time
        self.assertEqual(forward, 10 / FORWARD_SPEED_CM_S)
        self.assertEqual(reverse, 10 / REVERSE_SPEED_CM_S * REVERSE_PENALTY)

    def test_b2_and_b3_complete_five_of_five_on_two_layouts(self):
        for layout in LAYOUTS:
            engine = CarRouteEngine(Pose(20, 20, math.pi / 2), layout)
            engine.b2.toggle()
            while engine.b2.running:
                engine.b2.step()
            self.assertEqual(engine.b2.completed_targets, 5)
            engine.toggle()
            while engine.running:
                engine.step()
            self.assertTrue(engine.finished)
            self.assertEqual(engine.completed_targets, 5)
            self.assertEqual(engine.visited_ids, engine.route_order)
            self.assertEqual(engine.pose, engine.route_targets[-1])
            self.assertTrue(any(segment.command.startswith("B") for segment in engine.executable_segments))
            for path, target in zip(engine.route_paths, engine.route_targets):
                self.assertEqual(path[-1], target)
                self.assertTrue(all(footprint_is_safe(pose, layout) for pose in path))


if __name__ == "__main__":
    unittest.main()
