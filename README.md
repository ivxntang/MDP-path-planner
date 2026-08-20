# MDP Path Planner - B.1 Simulator

This is the first deliverable for the SC2079 Robot Path Planning module.

It demonstrates checklist item **B.1 - Robot Movement Area Simulator**:

- 2 m x 2 m exploration arena displayed as a 20 x 20 grid
- start zone, five obstacles and their image-facing sides
- safe viewing targets for each image
- robot pose and heading
- animated forward, backward and curved turning movement

## Run it in VS Code

1. Open the `mdp-path-planner` folder in VS Code.
2. Open the integrated terminal.
3. Run:

   ```bash
   python3 main.py
   ```

`tkinter` is included with the macOS system Python, so there are no project packages to install.

If a Homebrew Python reports that `_tkinter` is unavailable, either run:

```bash
/usr/bin/python3 main.py
```

or install the Homebrew `python-tk` formula that matches your Python version.

## Controls

| Key | Action |
| --- | --- |
| `Space` | Play or pause the complete planned route |
| `P` | Play or pause the complete planned route |
| `O` | Play or pause the exhaustive B.3 shortest-time route |
| `D` | Play or pause the short sample movement demo |
| `R` | Reset the robot to the start zone |
| `Up` | Move robot forward 5 cm |
| `Down` | Move robot backward 5 cm |
| `Left` | Make a 15 degree left curved movement |
| `Right` | Make a 15 degree right curved movement |
| `Esc` | Close the simulator |

## File guide

- `models.py`: simple data types such as `Pose` and `Obstacle`.
- `targets.py`: converts an obstacle's image side into the position and direction at which the robot should recognise it.
- `motion.py`: moves the robot in straight lines and minimum-radius turns.
- `simulator.py`: draws the grid map and animation.
- `main.py`: contains the sample obstacle map and launches the application.
- `b3_optimizer.py`: exhaustively scores all 120 safe target orders and plays the best route.

## What to do next

1. Replace the five sample obstacles in `main.py` with your team's test layout.
2. Confirm the coloured image-side line and green viewing target are correct for each obstacle.
3. Demonstrate B.1 to your supervisor.
4. Next, add collision checking and route planning in new files. Do not begin Android, Bluetooth or physical robot integration before this simulation is stable.
