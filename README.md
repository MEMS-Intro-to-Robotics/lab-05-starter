# Lab 5: Arm Control with MoveIt (Kinova Gen3 Lite, Simulation): [Your Name]

ECE 383 / ME 555: Introduction to Robotics and Automation (Fall 2026)

Update this README with your name, NetID, and a 1–3 line summary of your work.

## Contents

- `scaffolds/`: the starting files for your package
  - `motion_planner.py`: milestone 1 is written out, apart from choosing your
    own retract configuration; milestones 2 to 4 are yours
  - `m5_detour.py`: the milestone 5 scaffold
  - `pen.py`: complete; draws the gripper's actual path in RViz. Do not edit it.
  - `table.py`: complete; puts the table under the arm in Gazebo and in MoveIt.
    Do not edit it.
- `lab05.rviz`: the RViz layout for this lab, with the pen's line already set up
- `docs/`: your five milestone screenshots
- `test_lab_5.py`: automated repository checks
- `pytest.ini`: limits `pytest` to `test_lab_5.py` so it skips the ROS 2 workspace

Create `ros2_ws/src/lab05_moveit/` by following the lab manual, then copy each
file from `scaffolds/` into the package's `scripts/` folder. Leave the originals
in `scaffolds/` so you keep a clean copy to fall back on.

## Run the grading checks

From the repository root on the VM:

```bash
pytest -v
```

Run it from the repository root, not from `ros2_ws/`. Started anywhere else,
`pytest` collects the test files that `ros2 pkg create` generated inside your
package and reports errors about `ament_copyright` and `ament_flake8` instead of
checking your submission.

The checks confirm that the repository contains your package, the three script
files, five readable images, an updated README, and entry points for all three
executables. They also flag tracked ROS 2 build output. Screenshot filenames and
the executable names in `setup.py` are recommendations, so reasonable
alternatives still pass.

Passing the repository checks does not mean the full lab has been graded.
Course staff use your Gradescope PDF to evaluate what your milestones do, the
evidence in each screenshot, and your discussion answers.

Failures are expected while the lab is incomplete. Fix every failure before the
final push and confirm that Classroom 50 reports a passing result.
