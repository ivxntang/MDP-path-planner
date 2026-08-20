"""Physical dimensions used by the first simulator."""

ARENA_CM = 200
GRID_CELL_CM = 10
OBSTACLE_CM = 10
ROBOT_CM = 30
TURN_RADIUS_CM = 25
VIEW_DISTANCE_CM = 25

# Purely visual settings.
SCALE = 3
MARGIN_PX = 35
PANEL_WIDTH_PX = 310
ANIMATION_STEP_CM = 1.5

# B.3 estimated-time model. Distance is measured along the safe A* waypoints.
FORWARD_SPEED_CM_S = 20.0
HEADING_CHANGE_PENALTY_S = 1.0
