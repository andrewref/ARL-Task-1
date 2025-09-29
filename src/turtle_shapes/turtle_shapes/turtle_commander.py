import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import String
import math
import time

def get_arrow_points(scale=7.0):
    points = []
    center_x = 5.5
    center_y = 5.5
    
    tail_w = scale * 0.5
    tail_h = scale * 0.1
    head_len = scale * 0.2
    
    tail_start_x = center_x - tail_w / 2.0
    tail_end_x = center_x + tail_w / 2.0
    
    tail_points = [
        (tail_start_x, center_y + tail_h),
        (tail_end_x, center_y + tail_h),
        (tail_end_x, center_y - tail_h),
        (tail_start_x, center_y - tail_h),
        (tail_start_x, center_y + tail_h)
    ]
    points.extend(tail_points)
    
    head_base_x = tail_end_x
    
    head_points = [
        (head_base_x, center_y + head_len),
        (head_base_x + head_len, center_y),
        (head_base_x, center_y - head_len),
        (head_base_x, center_y + head_len)
    ]
    
    points.append(None)
    
    points.append(head_points[0])
    points.append(None)
    
    points.extend(head_points)
    
    return points

def get_star_points(R=4.5):
    points = []
    center_x = 5.5
    center_y = 5.5
    
    star_r = R * 0.8
    angle_offset = -math.pi/2 
    
    star_vertices = []
    for i in range(5):
        angle = angle_offset + i * 2 * math.pi / 5
        x = center_x + star_r * math.cos(angle)
        y = center_y + star_r * math.sin(angle)
        star_vertices.append((float(x), float(y)))
    
    star_path = [
        star_vertices[0],
        star_vertices[2],
        star_vertices[4],
        star_vertices[1],
        star_vertices[3],
        star_vertices[0]
    ]
    
    points.extend(star_path)
    
    return points

def get_flower_points(R=4.5, num_petals=5, num_segments=50):
    points = []
    center_x = 5.5
    center_y = 5.5
    
    circle_r = 0.5 * R / 4.5 
    
    points.append((float(center_x + circle_r), float(center_y)))

    for i in range(num_segments + 1):
        angle = 2 * math.pi * i / num_segments
        x = center_x + circle_r * math.cos(angle)
        y = center_y + circle_r * math.sin(angle)
        points.append((float(x), float(y)))
    
    points.append(None) 

    petal_len = 1.5 * R / 4.5 
    
    for i in range(num_petals):
        angle = i * 2 * math.pi / num_petals
        
        base_x1 = center_x + circle_r * math.cos(angle + 0.3)
        base_y1 = center_y + circle_r * math.sin(angle + 0.3)
        base_x2 = center_x + circle_r * math.cos(angle - 0.3)
        base_y2 = center_y + circle_r * math.sin(angle - 0.3)
        
        tip_x = center_x + (circle_r + petal_len) * math.cos(angle)
        tip_y = center_y + (circle_r + petal_len) * math.sin(angle)

        points.append(None) 
        points.append((float(base_x1), float(base_y1)))

        petal_path = [
            (base_x1, base_y1),
            (tip_x, tip_y),
            (base_x2, base_y2),
            (base_x1, base_y1)
        ]
        points.extend(petal_path)
        
        points.append(None) 
        

    return points


class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')

        self.create_subscription(String, '/selected_shape', self.shape_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        self.get_logger().info("Waiting for services...")
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service not available, waiting again...')
        self.get_logger().info("Services ready!")

        self.current_pose = None
        self.path_points = []
        self.point_index = 0
        self.state = 'idle'
        self.target_x = 0.0
        self.target_y = 0.0
        
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.stuck_counter = 0

        self.create_timer(0.02, self.control_loop) 

    def pose_callback(self, msg):
        if self.current_pose:
            self.prev_x = self.current_pose.x
            self.prev_y = self.current_pose.y
        self.current_pose = msg

    def shape_callback(self, msg):
        if self.state != 'idle' and msg.data != '0':
            self.get_logger().warn("Turtle is currently moving. Please wait or send '0' to stop.")
            return

        shape = msg.data.strip()
        if shape == '0':
            self.stop_drawing()
        else:
            self.draw_shape(shape)

    def draw_shape(self, shape):
        if shape == '1':
            self.path_points = get_arrow_points()
            self.get_logger().info("Starting Arrow (1)")
        elif shape == '2':
            self.path_points = get_star_points()
            self.get_logger().info("Starting Star (2)")
        elif shape == '3':
            self.path_points = get_flower_points()
            self.get_logger().info("Starting Flower (3)")
        else:
            self.get_logger().warn("Unknown shape command received.")
            return

        x0, y0 = self.path_points[0]
        x1, y1 = self.path_points[1]
        start_angle = math.atan2(y1 - y0, x1 - x0)

        self.set_pen_request(0, 0, 0, 3, 1)
        
        self.teleport_client.call_async(self._create_teleport_request(x0, y0, start_angle))
        
        time.sleep(0.1) 

        self.set_pen_request(0, 0, 0, 3, 0)

        self.point_index = 1
        self.target_x, self.target_y = self.path_points[self.point_index]
        self.state = 'moving'
        self.stuck_counter = 0

    def _create_teleport_request(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        return req

    def set_pen_request(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)

    def check_for_stuck_and_recover(self, current_twist):
        
        if math.sqrt((self.current_pose.x - self.prev_x)**2 + (self.current_pose.y - self.prev_y)**2) < 0.005:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            
        if self.stuck_counter > 5:
            self.get_logger().warn("Turtle might be stuck. Attempting recovery.")
            current_twist.linear.x = 2.0 
            current_twist.angular.z = current_twist.angular.z * 1.5 
            self.stuck_counter = 0
            return True
        return False
        
    def control_loop(self):
        if self.state != 'moving' or not self.current_pose:
            return

        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        dist = math.sqrt(dx**2 + dy**2)

        twist = Twist()

        if dist < 0.05: 
            self.next_point()
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - self.current_pose.theta),
                                math.cos(target_angle - self.current_pose.theta))

        angular_vel = 6.0 * angle_diff
        linear_vel = 4.0 * dist 

        twist.angular.z = max(min(angular_vel, 5.0), -5.0)
        twist.linear.x = max(min(linear_vel, 4.0), 0.0)

        if abs(angle_diff) > 0.5:
             twist.linear.x = 1.0 
        elif abs(angle_diff) > 0.1:
             twist.linear.x = min(twist.linear.x, 2.0)
        
        self.check_for_stuck_and_recover(twist)

        self.cmd_vel_pub.publish(twist)

    def next_point(self):
        self.point_index += 1
        if self.point_index >= len(self.path_points):
            self.finish_drawing()
            return

        if self.path_points[self.point_index] is None:
            if self.path_points[self.point_index - 1] is None:
                self.set_pen_request(0, 0, 0, 3, 0) 
            else:
                self.set_pen_request(0, 0, 0, 3, 1) 
            
            self.point_index += 1
            if self.point_index >= len(self.path_points):
                self.finish_drawing()
                return

        self.target_x, self.target_y = self.path_points[self.point_index]
        self.stuck_counter = 0

    def finish_drawing(self):
        self.set_pen_request(0, 0, 0, 3, 1)
        self.cmd_vel_pub.publish(Twist())
        self.state = 'idle'
        self.get_logger().info("Finished drawing")

    def stop_drawing(self):
        self.set_pen_request(0, 0, 0, 3, 1)
        self.cmd_vel_pub.publish(Twist())
        self.state = 'idle'
        self.get_logger().info("Turtle stopped")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()