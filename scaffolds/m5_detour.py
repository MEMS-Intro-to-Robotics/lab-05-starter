"""Milestone 5: make the planner route around an obstacle, and show that it did.

Copy this file into your package before you edit it:

    cp scaffolds/m5_detour.py ros2_ws/src/lab05_moveit/lab05_moveit/scripts/

Choose the start, goal, obstacle, and motion type. The manual gives the
requirements and the evidence to collect. The helpers below are the same ones
used in milestone 1.

    ros2 run lab05_moveit pen        # in its own terminal, first
    ros2 run lab05_moveit m5_detour
"""

import os
import signal
import threading
import time

import rclpy
from moveit_msgs.srv import GetPositionFK
from pymoveit2 import MoveIt2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from lab05_moveit.scripts.pen import PenClient
from lab05_moveit.scripts.table import add_table

# TODO: Put your NetID here so it appears in your screenshots.
NETID = os.getenv("NETID", "your_netid").strip()

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class M5DetourNode(Node):
    def __init__(self) -> None:
        super().__init__("m5_detour_node")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        self.pen = PenClient(self)
        self._fk_client = self.create_client(GetPositionFK, "compute_fk")
        self.get_logger().info(f"Lab 5 milestone 5 started ({NETID}).")

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """Wait for MoveIt to answer before sending it anything."""
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
        self.get_logger().error("MoveIt answered but could not compute the arm's pose.")
        return False

    def move_to_joints(self, joint_positions: list[float], attempts: int = 3) -> bool:
        """Plan and execute a joint-space motion, retrying a failed plan.

        Planning is randomized, so one attempt can fail even when a detour exists.
        Retry before moving the box.
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
        self.get_logger().error("Could not plan or execute this motion.")
        return False

    def run(self) -> None:
        # TODO: Record the path the arm takes with no obstacle in the way.
        # Choose a pen color for it with self.pen.set_color(red, green, blue),
        # put the pen down, run the motion, pen up.

        # TODO: Add a collision box positioned so the planner cannot use that
        # path any more. Wait about a second afterwards for the planning scene
        # to update.

        # TODO: Run the same motion again in a second pen color, so one
        # screenshot shows both paths.

        # TODO: Remove the collision box so the scene is clean for the next run.

        # TODO: Replace this line with the evidence the manual asks for: success,
        # planning attempts, and duration for each motion.
        self.get_logger().info("m5_detour finished.")


def main(args: list[str] | None = None) -> None:
    # Handle Ctrl+C ourselves; see the note in motion_planner.py.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node: M5DetourNode | None = None

    def stop(_signum: int, _frame: object) -> None:
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        if node is not None:
            try:
                node.pen.up()
            except Exception:  # noqa: BLE001 - nothing useful to do while quitting
                pass
        os._exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    node = M5DetourNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        if node.wait_until_ready():
            add_table(node, node.moveit2)
            time.sleep(1.0)  # let the planning scene update
            node.run()
    finally:
        node.pen.up()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
