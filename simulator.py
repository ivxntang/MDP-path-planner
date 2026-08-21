"""Tkinter visualiser for SC2079 checklist B.1."""

import math
import tkinter as tk
from typing import Iterable, Optional, Tuple

from config import (
    ANIMATION_STEP_CM,
    ARENA_CM,
    GRID_CELL_CM,
    FORWARD_SPEED_CM_S,
    MARGIN_PX,
    OBSTACLE_CM,
    PANEL_WIDTH_PX,
    ROBOT_CM,
    ROBOT_LENGTH_CM,
    ROBOT_WIDTH_CM,
    REVERSE_PENALTY,
    REVERSE_SPEED_CM_S,
    SAFETY_MARGIN_CM,
    SCALE,
    TURN_RADIUS_CM,
    TURNING_SPEED_CM_S,
)
from models import Obstacle, Pose
from motion import move_curve, move_straight
from map_editor import MapEditorModel
from physical_planner import CarRouteEngine
from route_engine import RouteEngine, point_in_virtual_obstacle
from targets import viewing_target


class MDPSimulator:
    def __init__(self, start_pose: Pose, obstacles: Iterable[Obstacle]) -> None:
        self.start_pose = start_pose
        self.pose = Pose(start_pose.x, start_pose.y, start_pose.theta)
        self.obstacles = list(obstacles)
        self.demo_running = False
        self.demo_after_id: Optional[int] = None
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.demo_commands = [
            ("F", 45.0),
            ("L", math.radians(45)),
            ("F", 50.0),
            ("R", math.radians(90)),
            ("F", 35.0),
            ("B", 18.0),
        ]

        self.root = tk.Tk()
        self.root.title("SC2079 MDP - B.1 Robot Movement Area Simulator")
        arena_px = ARENA_CM * SCALE
        self.canvas = tk.Canvas(
            self.root,
            width=arena_px + MARGIN_PX * 2 + PANEL_WIDTH_PX,
            height=arena_px + MARGIN_PX * 2,
            background="#f8fafc",
            highlightthickness=0,
        )
        self.canvas.pack()
        self.root.bind("<space>", self.toggle_planned_route)
        self.root.bind("d", self.toggle_demo)
        self.root.bind("D", self.toggle_demo)
        self.root.bind("r", self.reset)
        self.root.bind("R", self.reset)
        self.root.bind("p", self.toggle_planned_route)
        self.root.bind("P", self.toggle_planned_route)
        self.root.bind("<Up>", lambda _event: self.manual_straight(5))
        self.root.bind("<Down>", lambda _event: self.manual_straight(-5))
        self.root.bind("<Left>", lambda _event: self.manual_curve(math.radians(15)))
        self.root.bind("<Right>", lambda _event: self.manual_curve(-math.radians(15)))
        self.root.bind("<Escape>", lambda _event: self.root.destroy())
        self.root.bind("3", self.toggle_car_route)
        self.canvas.bind("<ButtonPress-1>", self.begin_obstacle_drag)
        self.canvas.bind("<ButtonRelease-1>", self.end_obstacle_drag)
        self.canvas.bind("<Button-3>", self.cycle_obstacle_face)

        self.editor = MapEditorModel(self.obstacles)
        self.dragged_obstacle_id = None
        self.edit_error = ""

        self.route_engine = RouteEngine(self.start_pose, self.obstacles)
        self.route_order = self.route_engine.route_order
        self.route_targets = self.route_engine.route_targets
        self.route_distance = self.route_engine.total_distance
        self.route_order_map = {obstacle_id: index + 1 for index, obstacle_id in enumerate(self.route_order)}
        self.route_index = self.route_engine.route_index
        self.route_running = self.route_engine.running
        self.route_after_id = None
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        self.car_engine = CarRouteEngine(self.start_pose, self.obstacles)
        self.car_after_id = None

        self.redraw()

    def world_to_screen(self, x: float, y: float) -> Tuple[float, float]:
        """Map bottom-left world coordinates to top-left screen coordinates."""

        sx = MARGIN_PX + x * SCALE
        sy = MARGIN_PX + (ARENA_CM - y) * SCALE
        return sx, sy

    def redraw(self) -> None:
        self.canvas.delete("all")
        self.draw_arena()
        self.draw_start_zone()
        self.draw_obstacles_and_targets()
        self.draw_robot()
        self.draw_info_panel()

    def draw_arena(self) -> None:
        left, top = self.world_to_screen(0, ARENA_CM)
        right, bottom = self.world_to_screen(ARENA_CM, 0)
        self.canvas.create_rectangle(left, top, right, bottom, fill="#ffffff", outline="#0f172a", width=3)

        for coordinate in range(0, ARENA_CM + 1, GRID_CELL_CM):
            x1, y1 = self.world_to_screen(coordinate, 0)
            x2, y2 = self.world_to_screen(coordinate, ARENA_CM)
            self.canvas.create_line(x1, y1, x2, y2, fill="#e2e8f0")
            x3, y3 = self.world_to_screen(0, coordinate)
            x4, y4 = self.world_to_screen(ARENA_CM, coordinate)
            self.canvas.create_line(x3, y3, x4, y4, fill="#e2e8f0")

        self.canvas.create_text(left, bottom + 18, text="0 cm", fill="#475569", anchor="w")
        self.canvas.create_text(right, bottom + 18, text="200 cm", fill="#475569", anchor="e")

    def draw_start_zone(self) -> None:
        left, top = self.world_to_screen(0, 40)
        right, bottom = self.world_to_screen(40, 0)
        self.canvas.create_rectangle(left, top, right, bottom, fill="#dbeafe", outline="#2563eb", width=2)
        self.canvas.create_text((left + right) / 2, (top + bottom) / 2, text="START\n40 x 40 cm", fill="#1d4ed8")

    def draw_obstacles_and_targets(self) -> None:
        # Draw the collision-safe A* path the robot actually follows.  Connecting
        # only the viewing targets produces misleading lines through obstacles.
        route_points = []
        for path in self.route_engine.route_paths:
            for waypoint in path:
                point = self.world_to_screen(waypoint.x, waypoint.y)
                if not route_points or point != route_points[-1]:
                    route_points.append(point)
        if len(route_points) >= 2:
            coordinates = [coordinate for point in route_points for coordinate in point]
            self.canvas.create_line(*coordinates, fill="#7c3aed", width=3, smooth=False)

        # B.3 selection (target order) and its non-holonomic executable path.
        selected = [self.world_to_screen(self.start_pose.x, self.start_pose.y)]
        selected.extend(self.world_to_screen(target.x, target.y) for target in self.car_engine.route_targets)
        if len(selected) >= 2:
            self.canvas.create_line(*[value for point in selected for value in point], fill="#16a34a", width=2, dash=(7, 5))
        executable = [self.world_to_screen(pose.x, pose.y) for pose in self.car_engine.executable_path]
        if len(executable) >= 2:
            self.canvas.create_line(*[value for point in executable for value in point], fill="#06b6d4", width=3)

        for obstacle in self.obstacles:
            clearance = max(ROBOT_LENGTH_CM, ROBOT_WIDTH_CM) / 2 + SAFETY_MARGIN_CM
            virtual_left, virtual_top = self.world_to_screen(obstacle.x - clearance, obstacle.y + OBSTACLE_CM + clearance)
            virtual_right, virtual_bottom = self.world_to_screen(obstacle.x + OBSTACLE_CM + clearance, obstacle.y - clearance)
            self.canvas.create_rectangle(
                virtual_left,
                virtual_top,
                virtual_right,
                virtual_bottom,
                fill="",
                outline="#ef4444",
                width=2,
            )

            left, top = self.world_to_screen(obstacle.x, obstacle.y + OBSTACLE_CM)
            right, bottom = self.world_to_screen(obstacle.x + OBSTACLE_CM, obstacle.y)
            self.canvas.create_rectangle(left, top, right, bottom, fill="#475569", outline="#0f172a", width=2)
            self.canvas.create_text((left + right) / 2, (top + bottom) / 2, text=str(obstacle.obstacle_id), fill="white", font=("Arial", 12, "bold"))

            # The thick orange line marks the image side.
            side = obstacle.image_side.upper()
            if side == "N":
                line = (left, top, right, top)
            elif side == "S":
                line = (left, bottom, right, bottom)
            elif side == "E":
                line = (right, top, right, bottom)
            else:
                line = (left, top, left, bottom)
            self.canvas.create_line(*line, fill="#f97316", width=5)

            target = viewing_target(obstacle)
            tx, ty = self.world_to_screen(target.x, target.y)
            self.canvas.create_oval(tx - 6, ty - 6, tx + 6, ty + 6, fill="#16a34a", outline="white", width=2)
            order = self.route_order_map.get(obstacle.obstacle_id)
            if order is not None:
                label = str(order)
                fill = "#1d4ed8" if obstacle.obstacle_id in self.visited_targets else "#15803d"
                self.canvas.create_text(tx, ty - 15, text=label, fill=fill, font=("Arial", 9, "bold"))
                if obstacle.obstacle_id in self.visited_targets:
                    self.canvas.create_text(tx, ty + 15, text="VISITED", fill="#7c3aed", font=("Arial", 8, "bold"))
            else:
                self.canvas.create_text(tx, ty - 15, text=f"T{obstacle.obstacle_id}", fill="#15803d", font=("Arial", 9, "bold"))

    def draw_robot(self) -> None:
        cx, cy = self.world_to_screen(self.pose.x, self.pose.y)
        half = ROBOT_CM * SCALE / 2
        cosine = math.cos(self.pose.theta)
        sine = math.sin(self.pose.theta)
        corners = []
        for local_x, local_y in ((-half, -half), (half, -half), (half, half), (-half, half)):
            corners.extend(
                (
                    cx + local_x * cosine + local_y * sine,
                    cy - local_x * sine + local_y * cosine,
                )
            )
        reversing = self.car_engine.running and self.car_engine.active_command.startswith("B")
        self.canvas.create_polygon(
            *corners,
            fill="#f59e0b" if reversing else "#2563eb",
            outline="#92400e" if reversing else "#1e3a8a",
            width=2,
        )

        # Screen y increases downward, hence subtract sin(theta).
        tip_x = cx + half * 0.9 * math.cos(self.pose.theta)
        tip_y = cy - half * 0.9 * math.sin(self.pose.theta)
        self.canvas.create_line(cx, cy, tip_x, tip_y, fill="white", width=4, arrow=tk.LAST)
        self.canvas.create_text(cx, cy + half + 12, text="Robot", fill="#1e3a8a", font=("Arial", 9, "bold"))

    def draw_info_panel(self) -> None:
        panel_x = MARGIN_PX + ARENA_CM * SCALE + 25
        heading_deg = (math.degrees(self.pose.theta) + 360) % 360
        route_summary = "No route planned"
        if self.route_order:
            route_summary = " -> ".join(str(item) for item in self.route_order)
        completion_text = f"Completed: {self.completed_targets}/{len(self.route_targets)} targets"
        commands = [segment.command for segment in self.car_engine.executable_segments]
        compressed_commands = []
        for command in commands:
            if not compressed_commands or compressed_commands[-1][0] != command:
                compressed_commands.append([command, 1])
            else:
                compressed_commands[-1][1] += 1
        command_text = " ".join(f"{command}x{count}" for command, count in compressed_commands)
        text = (
            "B.1 SIMULATOR\n\n"
            "Orange edge = image face\n"
            "Green dot = viewing target\n\n"
            "Red outline = forbidden safety zone\n"
            "Purple solid = B.2 grid route\n"
            "Green dashed = B.3 selected route\n"
            "Cyan solid = executable car route\n\n"
            f"Robot x: {self.pose.x:.1f} cm\n"
            f"Robot y: {self.pose.y:.1f} cm\n"
            f"Heading: {heading_deg:.0f} degrees\n\n"
            "B.2 ROUTE\n"
            f"Visit order: {route_summary}\n"
            f"Total distance: {self.route_distance:.1f} cm\n"
            f"{completion_text}\n\n"
            "B.3 TIME-OPTIMISED CAR ROUTE\n"
            f"Estimated time: {self.car_engine.estimated_travel_time:.1f} s\n"
            f"Active command: {self.car_engine.active_command}\n"
            f"Segments: {command_text}\n"
            "F/B = gear, L/R = steering\n"
            "Orange robot = reversing\n\n"
            "CONTROLS\n"
            "Space  Play/pause B.2 route\n"
            "P      Play/pause B.2 route\n"
            "3      Play/pause B.3 car route\n"
            "Drag   Move obstacle (5 cm snap)\n"
            "Right  Cycle obstacle image face\n"
            "D      Play/pause sample demo\n"
            "R      Reset\n"
            "Up     Forward 5 cm\n"
            "Down   Backward 5 cm\n"
            "Left   Curved left turn\n"
            "Right  Curved right turn\n"
            "Esc    Close"
        )
        self.canvas.create_text(panel_x, MARGIN_PX, text=text, anchor="nw", justify="left", fill="#0f172a", font=("Arial", 12))
        if self.edit_error:
            self.canvas.create_text(panel_x, MARGIN_PX + 500, text=self.edit_error, anchor="nw", fill="#dc2626", width=PANEL_WIDTH_PX - 30)

    def screen_to_world(self, screen_x: float, screen_y: float) -> Tuple[float, float]:
        return (screen_x - MARGIN_PX) / SCALE, ARENA_CM - (screen_y - MARGIN_PX) / SCALE

    def begin_obstacle_drag(self, event: object) -> None:
        x, y = self.screen_to_world(event.x, event.y)
        obstacle = self.editor.obstacle_at(x, y)
        self.dragged_obstacle_id = obstacle.obstacle_id if obstacle else None

    def end_obstacle_drag(self, event: object) -> None:
        if self.dragged_obstacle_id is None:
            return
        x, y = self.screen_to_world(event.x, event.y)
        obstacle = next(item for item in self.obstacles if item.obstacle_id == self.dragged_obstacle_id)
        # Preserve the pointer's initial convention by centring the obstacle at release.
        previous = list(self.editor.obstacles)
        if self.editor.move(self.dragged_obstacle_id, x - OBSTACLE_CM / 2, y - OBSTACLE_CM / 2):
            try:
                self.recompute_routes_after_edit()
            except ValueError as error:
                self.editor.obstacles = previous
                self.edit_error = str(error)
        else:
            self.edit_error = "Invalid placement: arena, start zone, or obstacle overlap"
        self.dragged_obstacle_id = None
        self.redraw()

    def cycle_obstacle_face(self, event: object) -> None:
        x, y = self.screen_to_world(event.x, event.y)
        obstacle = self.editor.obstacle_at(x, y)
        if obstacle and self.editor.cycle_face(obstacle.obstacle_id):
            previous_side = obstacle.image_side
            try:
                self.recompute_routes_after_edit()
            except ValueError as error:
                # Restore just the cycled face; positions are unchanged.
                while self.editor.obstacle_at(x, y).image_side != previous_side:
                    self.editor.cycle_face(obstacle.obstacle_id)
                self.edit_error = str(error)
            self.redraw()

    def recompute_routes_after_edit(self) -> None:
        self.stop_demo()
        self.stop_planned_route()
        if self.car_after_id is not None:
            self.root.after_cancel(self.car_after_id)
            self.car_after_id = None
        edited_obstacles = list(self.editor.obstacles)
        new_route_engine = RouteEngine(self.start_pose, edited_obstacles)
        new_car_engine = CarRouteEngine(self.start_pose, edited_obstacles)
        self.obstacles = edited_obstacles
        self.route_engine = new_route_engine
        self.car_engine = new_car_engine
        self.route_order = self.route_engine.route_order
        self.route_targets = self.route_engine.route_targets
        self.route_distance = self.route_engine.total_distance
        self.route_order_map = {obstacle_id: index + 1 for index, obstacle_id in enumerate(self.route_order)}
        self.route_index = 0
        self.visited_targets = set()
        self.completed_targets = 0
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.pose = Pose(self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        self.edit_error = ""

    def toggle_car_route(self, _event: Optional[object] = None) -> None:
        self.stop_demo()
        self.stop_planned_route()
        self.car_engine.toggle()
        if self.car_engine.running:
            self.car_after_id = self.root.after(30, self.animate_car_route)

    def stop_car_route(self) -> None:
        self.car_engine.running = False
        if self.car_after_id is not None:
            self.root.after_cancel(self.car_after_id)
            self.car_after_id = None

    def animate_car_route(self) -> None:
        self.car_after_id = None
        self.pose = self.car_engine.step()
        self.visited_targets = set(self.car_engine.visited_ids)
        self.completed_targets = self.car_engine.completed_targets
        self.redraw()
        if self.car_engine.running:
            command = self.car_engine.active_command
            if "L" in command or "R" in command:
                speed = TURNING_SPEED_CM_S
            elif command.startswith("B"):
                speed = REVERSE_SPEED_CM_S / REVERSE_PENALTY
            else:
                speed = FORWARD_SPEED_CM_S
            delay_ms = round(30 * FORWARD_SPEED_CM_S / speed)
            self.car_after_id = self.root.after(delay_ms, self.animate_car_route)

    def is_valid_pose(self, pose: Pose) -> bool:
        if not (15 <= pose.x <= 185 and 15 <= pose.y <= 185):
            return False
        for obstacle in self.obstacles:
            if point_in_virtual_obstacle(pose.x, pose.y, [obstacle]):
                return False
        return True

    def manual_straight(self, distance: float) -> None:
        self.stop_demo()
        self.stop_planned_route()
        candidate = move_straight(self.pose, distance)
        if self.is_valid_pose(candidate):
            self.pose = candidate
        self.redraw()

    def manual_curve(self, angle: float) -> None:
        self.stop_demo()
        self.stop_planned_route()
        candidate = move_curve(self.pose, angle)
        if self.is_valid_pose(candidate):
            self.pose = candidate
        self.redraw()

    def stop_demo(self) -> None:
        self.demo_running = False
        if self.demo_after_id is not None:
            self.root.after_cancel(self.demo_after_id)
            self.demo_after_id = None

    def stop_planned_route(self) -> None:
        self.route_engine.running = False
        self.route_running = False
        if self.route_after_id is not None:
            self.root.after_cancel(self.route_after_id)
            self.route_after_id = None

    def reset(self, _event: Optional[object] = None) -> None:
        self.stop_demo()
        self.stop_planned_route()
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.route_engine.reset()
        self.car_engine.reset()
        self.route_index = self.route_engine.route_index
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        self.pose = self.route_engine.pose
        self.redraw()

    def toggle_demo(self, _event: Optional[object] = None) -> None:
        if self.demo_running:
            self.stop_demo()
            return

        self.stop_planned_route()
        # Pressing play after a completed route starts it again from the beginning.
        if self.demo_index >= len(self.demo_commands):
            self.demo_index = 0
            self.demo_remaining = 0.0
            self.pose = Pose(self.start_pose.x, self.start_pose.y, self.start_pose.theta)
        self.demo_running = True
        self.demo_after_id = self.root.after(20, self.animate_demo)

    def animate_demo(self) -> None:
        self.demo_after_id = None
        if not self.demo_running:
            return
        if self.demo_index >= len(self.demo_commands):
            self.demo_running = False
            return

        command, total = self.demo_commands[self.demo_index]
        if self.demo_remaining == 0:
            # Straight commands are measured in cm; turn commands are stored as
            # angles, so convert their arc to cm for constant-speed animation.
            self.demo_remaining = (
                abs(total)
                if command in ("F", "B")
                else abs(total) * TURN_RADIUS_CM
            )

        step = min(ANIMATION_STEP_CM, self.demo_remaining)
        if command == "F":
            candidate = move_straight(self.pose, step)
        elif command == "B":
            candidate = move_straight(self.pose, -step)
        elif command == "L":
            candidate = move_curve(self.pose, step / TURN_RADIUS_CM)
        elif command == "R":
            candidate = move_curve(self.pose, -step / TURN_RADIUS_CM)
        else:
            candidate = self.pose

        if self.is_valid_pose(candidate):
            self.pose = candidate

        self.demo_remaining -= step
        if self.demo_remaining <= 1e-6:
            self.demo_remaining = 0
            self.demo_index += 1
        self.redraw()
        self.demo_after_id = self.root.after(20, self.animate_demo)

    def toggle_planned_route(self, _event: Optional[object] = None) -> None:
        self.stop_demo()
        self.route_engine.toggle()
        self.route_running = self.route_engine.running and not self.route_engine.paused
        if self.route_after_id is not None:
            self.root.after_cancel(self.route_after_id)
            self.route_after_id = None
        if self.route_running:
            self.route_after_id = self.root.after(30, self.animate_planned_route)

    def animate_planned_route(self) -> None:
        self.route_after_id = None
        if not self.route_engine.running:
            self.route_running = False
            return
        previous_completed = self.route_engine.completed_targets
        self.pose = self.route_engine.step()
        self.route_index = self.route_engine.route_index
        self.route_running = self.route_engine.running
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        if self.completed_targets > previous_completed:
            target_id = self.route_engine.visited_ids[-1]
            print(f"Reached obstacle {target_id}; completed {self.completed_targets}/{len(self.route_targets)}")
        self.redraw()
        if not self.route_engine.running:
            return
        self.route_after_id = self.root.after(30, self.animate_planned_route)

    def run(self) -> None:
        self.root.mainloop()
