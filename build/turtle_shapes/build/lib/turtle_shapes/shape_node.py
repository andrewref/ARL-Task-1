import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, '/selected_shape', 10)
        self.get_logger().info("ShapeNode started. Type a shape (1=lissajous, 2=cardioid, 3=rose, 0=stop).")
        self.timer = self.create_timer(0.1, self.publish_input)
        self.shape_to_publish = None

    def publish_input(self):
        if self.shape_to_publish is None:
            shape = input("Enter shape: ")
            self.shape_to_publish = shape
            msg = String()
            msg.data = shape
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {shape}")
            self.shape_to_publish = None

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
