import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('command_executor_node')
        self.get_logger().info("Command Executor Node is ready to receive commands.")

        # Subscriber รับข้อความจาก topic "command_topic"
        self.subscription = self.create_subscription(
            String,
            'node_control_topic',  # ชื่อ topic ที่รับคำสั่ง
            self.execute_command_callback,
            10
        )

    def execute_command_callback(self, msg):
        command = msg.data  # รับคำสั่งจาก msg
        self.get_logger().info(f"Received command: {command}")

        # เช็คคำสั่ง "shutdown" เพื่อปิดโหนด
        if command.lower() == "shutdown":
            self.get_logger().info("Shutdown command received. Shutting down node.")
            rclpy.shutdown()  # สั่งให้โหนดหยุดทำงาน

        else:
            # รันคำสั่งที่ได้รับจากผู้ใช้
            try:
                result = subprocess.run(command.split(), capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info(f"Command output: {result.stdout}")
                else:
                    self.get_logger().error(f"Command error: {result.stderr}")
            except Exception as e:
                self.get_logger().error(f"Error executing command: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutorNode()

    # รัน node จนกว่าจะมีการหยุด
    rclpy.spin(node)

    # ทำลาย node และหยุดการทำงานของ ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
