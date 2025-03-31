import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
import threading
from ament_index_python.packages import get_package_share_directory

# Get the installed package's shared directory
PACKAGE_NAME = "ros2_mi_eeg_xyz_control"
WAYPOINTS_FILE = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "waypoints.yaml")

class WaypointSaver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')

        # Subscribe to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_positions = {}

        self.get_logger().info(f"Waypoint Saver Initialized. Saving to {WAYPOINTS_FILE}")

    def joint_state_callback(self, msg):
        """ Store latest joint states dynamically using a dictionary """
        self.current_joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}

    def save_waypoint(self, waypoint_name):
        """ Saves the current joint positions to a YAML file under the given name """
        if not self.current_joint_positions:
            self.get_logger().error("No joint states received yet!")
            return

        # Ensure joints are saved in the correct order
        joint_order = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        ordered_positions = [self.current_joint_positions.get(joint, 0.0) for joint in joint_order]

        # Load existing waypoints
        with open(WAYPOINTS_FILE, "r") as f:
            waypoints = yaml.safe_load(f) or {}

        # Save new waypoint
        waypoints[waypoint_name] = ordered_positions

        # Write back to file
        with open(WAYPOINTS_FILE, "w") as f:
            yaml.dump(waypoints, f, default_flow_style=False)

        self.get_logger().info(f"Saved waypoint '{waypoint_name}': {waypoints[waypoint_name]}")

def user_input_thread(node):
    """ Function to handle keyboard input in a separate thread """
    while rclpy.ok():
        user_input = input("\nEnter waypoint name to save (or 'exit' to quit): ").strip()
        if user_input.lower() == 'exit' or user_input.lower() == 'quit':
            node.destroy_node()
            rclpy.shutdown()
            break
        node.save_waypoint(user_input)

def main():
    rclpy.init()
    node = WaypointSaver()

    # Start user input in a separate thread
    input_thread = threading.Thread(target=user_input_thread, args=(node,), daemon=True)
    input_thread.start()

    # Keep ROS2 node running
    rclpy.spin(node)
    
    try:
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass
        
if __name__ == '__main__':
    main()

