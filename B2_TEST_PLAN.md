# B.2 Manual Test Plan

Run the simulator with `python3 main.py`.

1. Press `R`, then `P`. Confirm the robot starts moving autonomously from `(20, 20)`.
2. Confirm the panel changes visibly from `Completed: 0/5 targets` to `Completed: 5/5 targets`.
3. Confirm `VISITED` labels appear at the five green image-viewing targets in the planned visit order.
4. Press `P` during playback. Confirm the robot pauses without changing pose or completion count. Press `P` again and confirm it resumes.
5. Press `R` during playback. Confirm the route cancels, the robot returns to `(20, 20)` facing North, and completion resets to `0/5`.
6. Confirm the robot centre remains outside every red 40 cm x 40 cm virtual-obstacle outline throughout playback.
7. After the fifth target, confirm the robot stops exactly on the fifth green target and the panel remains at `Completed: 5/5 targets`.
