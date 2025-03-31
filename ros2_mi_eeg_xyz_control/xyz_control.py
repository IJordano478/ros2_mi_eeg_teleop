import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
import time

class MoveItCartesianController(Node):
    def __init__(self):
        super().__init__('moveit_cartesian_controller')

        # Initialize MoveItPy
        self.moveit = MoveItPy(node_name="moveit_cartesian_controller")

        # Get planning component for the arm
        self.arm = self.moveit.get_planning_component("arm")  # Ensure "arm" matches your planning group

        # Wait for MoveItPy to initialize
        time.sleep(2)
        self.get_logger().info("MoveItPy Initialized!")

    def move_to_xyz(self, x, y, z):
        """ Moves the robot to the specified Cartesian position using MoveIt2 """

        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0  # Keep default orientation

        # Plan a Cartesian path
        waypoints = [target_pose]
        (trajectory, fraction) = self.arm.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0.0)

        if fraction < 1.0:
            self.get_logger().warn(f"Cartesian path planning incomplete! Only {fraction*100}% computed.")

        # Execute the planned trajectory
        if trajectory:
            self.arm.execute(trajectory)
            self.get_logger().info(f"Moved to XYZ: ({x}, {y}, {z})")
        else:
            self.get_logger().error("Failed to plan a Cartesian path!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItCartesianController()

    # Move the robot to a test Cartesian position
    node.move_to_xyz(0.4, 0.2, 0.5)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

