#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import threading
import time
import select


class KeypressNode(Node):
    def __init__(self):
        super().__init__('keypress_node')
        self.publisher_ = self.create_publisher(String, 'keypress', 10)
        self.running = True
        self.current_key = None  # Store the current key being pressed
        self.last_logged_key = None  # Track the last key that was logged
        self.last_key_time = time.time()  # Time of the last keypress
        self.lock = threading.Lock()

        # Timer for publishing at 0.01s intervals
        self.timer = self.create_timer(0.01, self.publish_keypress)

        # Start a separate thread for key listening
        self.key_listener_thread = threading.Thread(target=self.listen_for_keypress)
        self.key_listener_thread.daemon = True
        self.key_listener_thread.start()

        self.get_logger().info('Keypress Node has started. Listening for keypresses...')

    def listen_for_keypress(self):
        """Continuously listens for keypresses and updates the current_key."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())  # Enable raw mode for real-time key detection
            while self.running:
                if select.select([sys.stdin], [], [], 0.01)[0]:  # Non-blocking key check
                    key = sys.stdin.read(1)
                    with self.lock:
                        if key == '\x03':  # Ctrl+C
                            self.running = False
                            break
                        self.current_key = key
                        self.last_key_time = time.time()  # Update last activity time
                else:
                    # No new key detected, but don't immediately mark as released
                    with self.lock:
                        if (time.time() - self.last_key_time) > 0.5:
                            self.current_key = None
        except KeyboardInterrupt:
            self.get_logger().info('KeyboardInterrupt detected. Shutting down...')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.get_logger().info('Key listener stopped.')

    def publish_keypress(self):
        """Publishes the currently pressed key at 0.01s intervals."""
        with self.lock:
            if self.current_key is not None:
                msg = String()
                msg.data = self.current_key
                self.publisher_.publish(msg)

                # Only log if the key changes (to prevent flooding logs)
                if self.current_key != self.last_logged_key:
                    sys.stdout.write(f'\rKeypress Published: {self.current_key}   ')
                    sys.stdout.flush()
                    self.last_logged_key = self.current_key
            else:
                if self.last_logged_key is not None:
                    sys.stdout.write(f'\rKey Released                ')
                    sys.stdout.flush()
                    self.last_logged_key = None

    def destroy_node(self):
        self.running = False
        self.key_listener_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeypressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Keypress Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


