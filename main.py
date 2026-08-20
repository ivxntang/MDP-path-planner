"""Run the first SC2079 Path Planning deliverable: B.1 simulator."""

import math

from models import Obstacle, Pose


def main() -> None:
    try:
        from simulator import MDPSimulator
    except ModuleNotFoundError as error:
        if error.name != "_tkinter":
            raise
        raise SystemExit(
            "This Python installation does not include tkinter. On macOS, run "
            "'/usr/bin/python3 main.py', or install the tkinter package matching "
            "your Python version."
        ) from None

    # Coordinates are the bottom-left corner of each 10 cm obstacle, in cm.
    # Change these later to match your team's Android-map test case.
    obstacles = [
        Obstacle(1, 60, 100, "N"),
        Obstacle(2, 130, 50, "W"),
        Obstacle(3, 150, 140, "S"),
        Obstacle(4, 80, 160, "E"),
        Obstacle(5, 40, 70, "N"),
    ]
    start_pose = Pose(20, 20, math.pi / 2)
    MDPSimulator(start_pose, obstacles).run()


if __name__ == "__main__":
    main()
