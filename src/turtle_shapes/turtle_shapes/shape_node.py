import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapePublisherNode(Node):
    def __init__(self):
        super().__init__('shape_publisher_node')
        self.pub = self.create_publisher(String, '/selected_shape', 10)
        self.get_logger().info("Shape Publisher ready!")

    def print_menu(self):
        self.get_logger().info("\n--- Turtle Shapes Menu ---")
        self.get_logger().info("1 = Arrow")
        self.get_logger().info("2 = Star")
        self.get_logger().info("3 = hot airballoon")
        self.get_logger().info("4 = Clear Screen")
        self.get_logger().info("0 = Stop Turtle")
        self.get_logger().info("--------------------------")

    def publish_shape(self, choice):
        if choice not in ['0', '1', '2', '3', '4']:
            self.get_logger().warn("Invalid choice. Type 0, 1, 2, 3, or 4")
            return

        msg = String()
        msg.data = choice
        self.pub.publish(msg)
        
        shape_names = {
            '0': 'Stop',
            '1': 'Arrow',
            '2': 'Star',
            '3': 'hotairballoon',
            '4': 'Clear Screen'
        }
        self.get_logger().info(f"Published: {shape_names[choice]}")

def main(args=None):
    rclpy.init(args=args)
    node = ShapePublisherNode()
    try:
        node.print_menu() 
        while rclpy.ok():
            try:
                choice = input("Enter shape number (0/1/2/3/4) or Ctrl+C to exit: ").strip()
                if not choice:
                    continue
                node.publish_shape(choice)
            except EOFError: 
                break 
            except KeyboardInterrupt:
                 raise

    except KeyboardInterrupt:
        node.get_logger().info("Shape Publisher Interrupted. Shutting down...")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()