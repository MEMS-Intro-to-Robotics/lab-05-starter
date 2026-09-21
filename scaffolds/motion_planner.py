"""Milestones 1 to 4: joint-space motion, Cartesian motion, the planning scene, and the gripper.

Copy this file into your package before you edit it:

    cp scaffolds/motion_planner.py ros2_ws/src/lab05_moveit/lab05_moveit/scripts/

Milestone 1 is written out for you, apart from choosing your own retract
configuration. Read it: every later milestone uses the same pattern of plan,
execute, wait, and check. Milestones 2 to 4 are yours to write, and the manual
states what each one has to do.

Run it with the simulation and MoveIt already running:

    ros2 run lab05_moveit motion_planner          # all milestones
    ros2 run lab05_moveit motion_planner 2        # only milestone 2
    ros2 run lab05_moveit motion_planner 2 3      # milestones 2 and 3

The pen draws where the gripper actually travels, so start it in another
terminal first: ros2 run lab05_moveit pen
"""

import os
import signal
import sys
import threading
import time

import rclpy
from moveit_msgs.srv import GetPositionFK
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from lab05_moveit.scripts.pen import PenClient

# TODO: Put your NetID here so it appears in your screenshots.
# You can also leave this alone and export NETID in your container instead.
NETID = os.getenv("NETID", "your_netid").strip()

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT = "right_finger_bottom_joint"
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class MotionPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_planner_node")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        # The finger sits at 0.0 fully open and closes as the value grows: 0.8
        # is a firm close. Gazebo clamps anything above 0.96 instead of
        # refusing it, so a larger number looks accepted but does nothing more.
        #
        # GripperInterface looks for the gripper action server as it is built,
        # before this node starts spinning, so it may log "Unable to determine
        # the appropriate interface for gripper". It checks again on the first
        # open() or close(), so that message on its own is not a problem.
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=[GRIPPER_JOINT],
            open_gripper_joint_positions=[0.0],
            closed_gripper_joint_positions=[0.8],
            gripper_group_name="gripper",
            gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
        )
        self.pen = PenClient(self)

        self._joint_positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._fk_client = self.create_client(GetPositionFK, "compute_fk")

        # Milestone 1 writes your retract configuration here so milestones 3 and
        # 5 can reuse it.
        self.retract_joints: list[float] | None = None

        self.get_logger().info(f"Lab 5 motion planner started ({NETID}).")

    # ---------------------------------------------------------------- helpers

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position

    def joint_position(self, name: str) -> float | None:
        """The latest measured position of one joint, or None before the first
        /joint_states message arrives."""
        return self._joint_positions.get(name)

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """Wait for MoveIt to answer before sending it anything.

        MoveIt takes several seconds to come up, and how long varies. A node
        that plans too early gets 'service not yet available' and its first call
        returns None.
        """
        if not self._fk_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"MoveIt did not start within {timeout:.0f} s. Is the sim.launch.py "
                "MoveIt launch still running in its own terminal?"
            )
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.moveit2.compute_fk() is not None:
                self.get_logger().info("MoveIt is ready.")
                return True
            time.sleep(1.0)
        self.get_logger().error(
            "MoveIt answered but could not compute the arm's current pose. Check "
            "the MoveIt terminal for errors."
        )
        return False

    def move_to_joints(self, joint_positions: list[float], attempts: int = 3) -> bool:
        """Plan and execute a joint-space motion, retrying a failed plan.

        Planning is randomized: the same request can fail once and succeed on the
        next attempt, especially with an obstacle nearby. A failed plan is not the
        same as an impossible one, so try again before you change anything.
        """
        for attempt in range(1, attempts + 1):
            trajectory = self.moveit2.plan(joint_positions=joint_positions)
            if trajectory is None:
                self.get_logger().warn(f"Planning attempt {attempt} of {attempts} failed.")
                continue
            self.moveit2.execute(trajectory)
            if self.moveit2.wait_until_executed():
                return True
            self.get_logger().warn(f"Execution attempt {attempt} of {attempts} failed.")
        self.get_logger().error(
            "Could not plan or execute this motion. Check that the goal is inside "
            "the joint limits and that nothing in the planning scene blocks it."
        )
        return False

    # ------------------------------------------------------- milestone 1 (done)

    def run_milestone_1(self) -> None:
        """Home, then your own retract configuration, in joint space."""
        self.get_logger().info("Milestone 1: joint-space motion")

        # TODO: Replace these angles (radians, joint_1 first) with a retract
        # configuration you chose yourself. Use the Joints tab of the RViz
        # MotionPlanning panel to find one that is collision free and clear of
        # the table, then read the six values back out.
        self.retract_joints = [0.0, -1.2, 1.4, 0.0, 1.1, 0.0]

        self.pen.clear()
        self.get_logger().info("Moving to Home.")
        self.move_to_joints(HOME)

        self.pen.down()
        self.get_logger().info("Moving to Retract.")
        reached = self.move_to_joints(self.retract_joints)
        self.pen.up()

        if reached:
            measured = [self.joint_position(name) for name in ARM_JOINTS]
            self.get_logger().info(
                "Retract reached. Measured joint positions: "
                + ", ".join(f"{value:+.3f}" if value is not None else "?" for value in measured)
            )

    # --------------------------------------------------- milestones 2, 3 and 4

    def run_milestone_2(self) -> None:
        """Two straight-line Cartesian segments. See the manual for the contract."""
        self.get_logger().info("Milestone 2: Cartesian motion")
        # TODO: Read the current end-effector pose, build each target pose from
        # it, and plan each segment with cartesian=True. Handle the case where
        # the planner returns None instead of a trajectory, and say which
        # segment failed. Put the pen down before the first segment and up after
        # the last one.

    def run_milestone_3(self) -> None:
        """A collision object the planner must respect. See the manual."""
        self.get_logger().info("Milestone 3: the planning scene")
        # TODO: Add a collision box to the planning scene, show that it changes
        # what the planner will do, then remove it and show the difference.

    def run_milestone_4(self) -> None:
        """Open and close the gripper, and prove it moved. See the manual."""
        self.get_logger().info("Milestone 4: gripper control")
        # TODO: Open and close the gripper, and read the finger joint back from
        # /joint_states to confirm it moved. self.joint_position(GRIPPER_JOINT)
        # returns the measured value.


def main(args: list[str] | None = None) -> None:
    # Handle Ctrl+C ourselves. rclpy's own handler tears the context down under
    # a motion that is still running, which leaves this node waiting for a
    # result that can no longer arrive.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node: MotionPlannerNode | None = None

    def stop(_signum: int, _frame: object) -> None:
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        if node is not None:
            # Lift the pen so the next run does not draw over this one.
            try:
                node.pen.up()
            except Exception:  # noqa: BLE001 - nothing useful to do while quitting
                pass
        os._exit(130)

    # Installed before the node is built: that construction takes a few seconds,
    # and Ctrl+C during it should still exit cleanly.
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    node = MotionPlannerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    milestones = {
        1: node.run_milestone_1,
        2: node.run_milestone_2,
        3: node.run_milestone_3,
        4: node.run_milestone_4,
    }
    # Anything from --ros-args onwards belongs to ROS, not to this script.
    own_args = sys.argv[1:]
    if "--ros-args" in own_args:
        own_args = own_args[: own_args.index("--ros-args")]

    requested: list[int] = []
    for argument in own_args:
        if argument.isdigit() and int(argument) in milestones:
            requested.append(int(argument))
        else:
            node.get_logger().error(
                f"'{argument}' is not a milestone in this file. Pass any of "
                "1, 2, 3, 4, or no arguments to run all four. Milestone 5 is "
                "m5_detour: ros2 run lab05_moveit m5_detour"
            )
            node.destroy_node()
            rclpy.try_shutdown()
            sys.exit(2)
    if not requested:
        requested = sorted(milestones)

    try:
        if node.wait_until_ready():
            for number in requested:
                milestones[number]()
                time.sleep(1.0)
    finally:
        node.pen.up()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
