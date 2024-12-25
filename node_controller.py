import rclpy
from rclpy.node import Node
import subprocess
import shlex
import signal
import os
from std_msgs.msg import String

class NodeController(Node):
    def __init__(self):
        super().__init__('node_controller')
        
        # Dictionary to keep track of running node processes
        self.running_nodes = {}
        
        # Subscriber to receive node control commands
        self.subscription = self.create_subscription(
            String,
            'node_control_topic',
            self.node_control_callback,
            10
        )
        
        # Publisher to send node control status
        self.status_publisher = self.create_publisher(
            String,
            'node_control_status',
            10
        )
        
        self.get_logger().info('Node Controller initialized')

    def node_control_callback(self, msg):
        """
        Handle node control commands
        Format of command: 'action:package_name:node_executable'
        Supported actions: 'start', 'stop', 'restart'
        """
        try:
            # Parse the incoming message
            parts = msg.data.split(':')
            if len(parts) != 3:
                self.send_status(f"Invalid command format. Use 'action:package_name:node_executable'. Received: {msg.data}")
                return
            
            action, package_name, node_executable = parts
            
            if action == 'start':
                self.start_node(package_name, node_executable)
            elif action == 'stop':
                self.stop_node(node_executable)
            elif action == 'restart':
                self.restart_node(package_name, node_executable)
            else:
                self.send_status(f"Unknown action: {action}")
        
        except Exception as e:
            self.send_status(f"Error processing command: {str(e)}")

    def start_node(self, package_name, node_executable):
        """Start a ROS2 node"""
        if node_executable in self.running_nodes:
            self.send_status(f"Node {node_executable} is already running")
            return
        
        try:
            # Construct the full command
            full_command = f"ros2 run {package_name} {node_executable}"
            
            # Use subprocess to start the node
            process = subprocess.Popen(
                shlex.split(full_command),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Allow killing the entire process group
            )
            
            self.running_nodes[node_executable] = process
            self.send_status(f"Started node: {package_name}/{node_executable}")
        
        except Exception as e:
            self.send_status(f"Failed to start node {package_name}/{node_executable}: {str(e)}")

    def stop_node(self, node_executable):
        """Stop a running ROS2 node"""
        if node_executable not in self.running_nodes:
            self.send_status(f"Node {node_executable} is not running")
            return
        
        try:
            # Get the process
            process = self.running_nodes[node_executable]
            
            # Send signal to the entire process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            
            # Wait for the process to terminate
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Force kill if not terminated
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            
            # Remove from running nodes
            del self.running_nodes[node_executable]
            
            self.send_status(f"Stopped node: {node_executable}")
        
        except Exception as e:
            self.send_status(f"Failed to stop node {node_executable}: {str(e)}")

    def restart_node(self, package_name, node_executable):
        """Restart a ROS2 node"""
        self.stop_node(node_executable)
        self.start_node(package_name, node_executable)

    def send_status(self, message):
        """Publish status messages"""
        status_msg = String()
        status_msg.data = message
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f'Status: {message}')

def main(args=None):
    rclpy.init(args=args)
    node_controller = NodeController()
    
    try:
        rclpy.spin(node_controller)
    except KeyboardInterrupt:
        node_controller.get_logger().info('Node stopped cleanly')
    finally:
        # Clean up running nodes
        for node_executable, process in node_controller.running_nodes.items():
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except:
                pass
        
        node_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()