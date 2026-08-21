"""Physical dimensions used by the first simulator."""

ARENA_CM = 200
GRID_CELL_CM = 10
OBSTACLE_CM = 10
# Physical robot configuration.  Keep the square-size alias for older B.1/B.2
# callers, but derive all new clearance geometry from length and width.
ROBOT_LENGTH_CM = 30
ROBOT_WIDTH_CM = 30
ROBOT_CM = ROBOT_WIDTH_CM
SAFETY_MARGIN_CM = 0
TARGET_STANDOFF_DISTANCE_CM = 20
MIN_TURNING_RADIUS_CM = 25
FORWARD_SPEED_CM_S = 20
REVERSE_SPEED_CM_S = 14
TURNING_SPEED_CM_S = 12
# Multiplier applied to reverse travel time.  Keep this close to one so that
# reversing remains available whenever it is quicker or needed for clearance.
REVERSE_PENALTY = 1.08

# Backwards-compatible names used by the original simulator.
TURN_RADIUS_CM = MIN_TURNING_RADIUS_CM
VIEW_DISTANCE_CM = OBSTACLE_CM / 2 + TARGET_STANDOFF_DISTANCE_CM

EDITOR_SNAP_CM = 5
START_ZONE_CM = 40
COLLISION_CHECK_STEP_CM = 1

# Purely visual settings.
SCALE = 3
MARGIN_PX = 35
PANEL_WIDTH_PX = 310
ANIMATION_STEP_CM = 1.5
