import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen
import math

class TurtleCommander(Node):

    def __init__(self):
        super().__init__('turtle_commander')
        self.subscription = self.create_subscription(
            String, '/selected_shape', self.shape_callback, 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clear_client = self.create_client(Empty, '/clear')  # <-- Fixed
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.current_pose = None
        self.state = 'idle'  # idle, turning, moving
        self.path_points = []
        self.point_index = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

    def pose_callback(self, msg):
        self.current_pose = msg

    def shape_callback(self, msg):
        if msg.data == '0':
            self.stop_drawing()
            return
        shape = msg.data
        self.draw_shape(shape)

    def draw_shape(self, shape):
        self.clear_screen()
        if shape == '1':
            self.path_points = self.generate_lissajous()
            self.get_logger().info('Drawing Lissajous figure-8')
        elif shape == '2':
            self.path_points = self.generate_cardioid()
            self.get_logger().info('Drawing Cardioid')
        elif shape == '3':
            self.path_points = self.generate_rose()
            self.get_logger().info('Drawing 4-petaled Rose')
        else:
            self.get_logger().warn('Unknown shape')
            return

        if not self.path_points:
            return

        # Compute initial theta to second point
        if len(self.path_points) > 1:
            dx = self.path_points[1][0] - self.path_points[0][0]
            dy = self.path_points[1][1] - self.path_points[0][1]
            initial_theta = math.atan2(dy, dx)
        else:
            initial_theta = 0.0

        self.teleport_to(self.path_points[0][0], self.path_points[0][1], initial_theta)
        self.set_pen(0.0, 0.0, 0.0, 3, 0)  # Pen down
        self.point_index = 1
        if self.point_index < len(self.path_points):
            self.target_x, self.target_y = self.path_points[self.point_index]
            self.state = 'turning'
        else:
            self.state = 'idle'

    def generate_lissajous(self):
        points = []
        num_points = 200
        for i in range(num_points + 1):
            t = 2 * math.pi * i / num_points
            x = 3.0 * math.sin(t)
            y = 3.0 * math.sin(2 * t)
            points.append((x + 5.5, y + 5.5))
        points.append(points[0])
        return points

    def generate_cardioid(self):
        points = []
        a = 1.5
        shift_x = 5.5 + 2 * a
        shift_y = 5.5
        num_points = 200
        for i in range(num_points + 1):
            t = 2 * math.pi * i / num_points
            x = 2 * a * (1 - math.cos(t)) * math.cos(t) + shift_x
            y = 2 * a * (1 - math.cos(t)) * math.sin(t) + shift_y
            points.append((x, y))
        points.append(points[0])
        return points

    def generate_rose(self):
        points = []
        a = 2.5
        num_points = 300
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            r = a * math.cos(2 * theta)
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append((x + 5.5, y + 5.5))
        points.append(points[0])
        return points

    def clear_screen(self):
        if not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Clear service not available')
            return
        req = Empty.Request()   # <-- Fixed for Humble
        future = self.clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def teleport_to(self, x, y, theta):
        if not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Teleport service not available')
            return
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def set_pen(self, r, g, b, width, off):
        if not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pen service not available')
            return
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def control_loop(self):
        if self.current_pose is None or self.state == 'idle':
            return
        if self.state == 'turning':
            self.turn_to_target()
        elif self.state == 'moving':
            self.move_to_target()

    def turn_to_target(self):
        target_angle = math.atan2(self.target_y - self.current_pose.y,
                                  self.target_x - self.current_pose.x)
        angle_diff = target_angle - self.current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        if abs(angle_diff) < 0.1:
            self.cmd_vel_pub.publish(Twist())
            self.state = 'moving'
        else:
            twist = Twist()
            twist.angular.z = max(min(2.0 * angle_diff, 2.0), -2.0)
            self.cmd_vel_pub.publish(twist)

    def move_to_target(self):
        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.1:
            self.next_point()
        else:
            twist = Twist()
            twist.linear.x = 2.0
            self.cmd_vel_pub.publish(twist)

    def next_point(self):
        self.point_index += 1
        if self.point_index >= len(self.path_points):
            self.finish_drawing()
            return
        self.target_x, self.target_y = self.path_points[self.point_index]
        self.state = 'turning'
        self.cmd_vel_pub.publish(Twist())

    def finish_drawing(self):
        self.set_pen(0.0, 0.0, 0.0, 3, 1)  # Pen up
        self.cmd_vel_pub.publish(Twist())
        self.state = 'idle'
        self.get_logger().info('Shape drawing complete')

    def stop_drawing(self):
        self.set_pen(0.0, 0.0, 0.0, 3, 1)  # Pen up
        self.cmd_vel_pub.publish(Twist())
        self.state = 'idle'
        self.get_logger().info('Turtle stopped')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
