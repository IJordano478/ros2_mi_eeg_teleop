import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time
import yaml
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')

        # Publisher to joint trajectory controller
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Action client for controlling the gripper
        self.gripper_client = ActionClient(
            self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd'
        )

        # Load waypoints from waypoints.yaml
        self.waypoints = self.load_waypoints()
        print(self.waypoints)

        # Subscriber to receive commands
        self.subscription = self.create_subscription(
            String,
            "UI/command_string",
            self.cmd_string_callback,
            10
        )

        self.current_gripper_state = "open"

        # Allow time for controller to start
        time.sleep(2)
        self.send_trajectory("hover", "open", force_gripper=True)  # Start at hover position
        print("Awaiting first instruction")
        time.sleep(5)

    def load_waypoints(self):
        """ Load waypoints from YAML file """
        package_name = "ros2_mi_eeg_xyz_control"
        waypoints_file = os.path.join(get_package_share_directory(package_name), "config", "waypoints.yaml")

        if not os.path.exists(waypoints_file):
            self.get_logger().error(f"Waypoints file not found: {waypoints_file}")
            return {}

        with open(waypoints_file, "r") as f:
            waypoints = yaml.safe_load(f) or {}

        self.get_logger().info(f"Loaded waypoints: {list(waypoints.keys())}")
        return waypoints

    def cmd_string_callback(self, msg: String):
        """ Handle incoming UI commands """
        if msg.data:
            cmd = msg.data.split(",")

            if cmd[1].lower() == "grab" and cmd[2].lower() == "block_a":
                 print("Running trajectory for:", cmd[1], cmd[2])
                 self.send_trajectory("block_a_1", "open")
                 time.sleep(5.5)
                 self.send_trajectory("block_a_2", "open")
                 time.sleep(5.5)
                 self.send_trajectory("block_a_2", "closed")
                 time.sleep(5.5)
                 self.send_trajectory("block_a_1", "closed")
                 time.sleep(5.5)
                 self.send_trajectory("hover", "closed")
                 time.sleep(5.5)

    def send_trajectory(self, waypoint_name, gripper_state, force_gripper=False):
        """ Publish a trajectory to move to the requested waypoint and control the gripper """
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f"Unknown waypoint: {waypoint_name}")
            return

        # Publish trajectory message
        traj = JointTrajectory()
        traj.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        
        traj_point = JointTrajectoryPoint()
        traj_point.positions = self.waypoints[waypoint_name]
        traj_point.time_from_start.sec = 5  # Move over 5 seconds
        traj.points.append(traj_point)

        self.get_logger().info(f"Moving to {waypoint_name} position...")
        self.trajectory_pub.publish(traj)

        # Wait for the motion to complete before controlling the gripper
        time.sleep(5.5)

        # Control the gripper (open/close)
        print(self.current_gripper_state, "to", gripper_state)
        if gripper_state != self.current_gripper_state or force_gripper:
            self.control_gripper(gripper_state)
            self.current_gripper_state = gripper_state

    def control_gripper(self, state):
        """ Sends a command to the gripper to open or close with a timeout """
        goal_msg = GripperCommand.Goal()

        if state == "open":
            goal_msg.command.position = 0.0  # Fully open
        elif state == "closed":
            goal_msg.command.position = 0.8  # Fully closed
        else:
            self.get_logger().error(f"Invalid gripper state: {state}")
            return

        self.get_logger().info(f"Setting gripper to {state}")

        # Wait for the action server (max 2 seconds)
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Gripper action server not available!")
            return

        # Send goal
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)

        # Wait for result with a timeout (max 5 seconds)
        timeout = 5.0  # Max wait time in seconds
        start_time = time.time()

        while not send_goal_future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error("Gripper action took too long! Moving on...")
                return
            time.sleep(0.1)  # Small delay to prevent CPU overload

        self.get_logger().info("Gripper action completed successfully.")

def main():
    rclpy.init()
    node = TrajectorySender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
