import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
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

def get_hotairballoon_points():
    points = []
    center_x = 5.5
    center_y = 5.5
    
    balloon_cx = center_x
    balloon_cy = center_y + 1.5
    balloon_radius = 1.6
    
    num_points = 24
    for i in range(num_points + 1):
        angle = i * 2 * math.pi / num_points
        x = balloon_cx + balloon_radius * math.cos(angle)
        y = balloon_cy + balloon_radius * math.sin(angle)
        points.append((float(x), float(y)))
    
    points.append(None)
    
    basket_width = 0.7
    basket_height = 0.45
    basket_cx = center_x
    basket_cy = center_y - 1.3
    
    basket_left = basket_cx - basket_width / 2
    basket_right = basket_cx + basket_width / 2
    basket_top = basket_cy + basket_height / 2
    basket_bottom = basket_cy - basket_height / 2
    
    points.extend([
        (basket_left, basket_top),
        (basket_right, basket_top),
        (basket_right, basket_bottom),
        (basket_left, basket_bottom),
        (basket_left, basket_top)
    ])
    
    points.append(None)
    
    rope_attach_y = balloon_cy - balloon_radius * 0.866
    
    points.extend([
        (balloon_cx - balloon_radius * 0.5, rope_attach_y),
        (basket_left + 0.05, basket_top)
    ])
    
    points.append(None)
    
    points.extend([
        (balloon_cx + balloon_radius * 0.5, rope_attach_y),
        (basket_right - 0.05, basket_top)
    ])
    
    return points

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')

        self.create_subscription(String, '/selected_shape', self.shape_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.clear_client = self.create_client(Empty, '/clear')

        self.get_logger().info("Waiting for services...")
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service not available, waiting again...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting again...')
        self.get_logger().info("Services ready!")

        self.current_pose = None
        self.path_points = []
        self.point_index = 0
        self.state = 'idle'
        self.target_x = 0.0
        self.target_y = 0.0
        self.pen_down = False
        self.stuck_counter = 0
        self.last_distance = 0.0
        self.progress_check_timer = 0

        self.create_timer(0.01, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def shape_callback(self, msg):
        shape = msg.data.strip()
        
        if shape == '0':
            self.stop_drawing()
            return
        elif shape == '4':
            self.clear_screen()
            return
        
        if self.state != 'idle':
            self.get_logger().warn("Turtle is currently moving. Please wait or send '0' to stop.")
            return

        self.draw_shape(shape)

    def draw_shape(self, shape):
        if shape == '1':
            self.path_points = get_arrow_points()
            self.get_logger().info("Starting Arrow")
        elif shape == '2':
            self.path_points = get_star_points()
            self.get_logger().info("Starting Star")
        elif shape == '3':
            self.path_points = get_hotairballoon_points()
            self.get_logger().info("Starting Hot Air Balloon")
        else:
            self.get_logger().warn(f"Unknown shape command: '{shape}'")
            return

        x0, y0 = self.path_points[0]
        
        self.set_pen(off=True)
        time.sleep(0.05)
        
        self.teleport_client.call_async(self._create_teleport_request(x0, y0, 0.0))
        time.sleep(0.1)
        
        self.set_pen(off=False)
        self.pen_down = True
        time.sleep(0.05)

        self.point_index = 1
        self.target_x, self.target_y = self.path_points[self.point_index]
        self.state = 'moving'
        self.stuck_counter = 0
        self.progress_check_timer = 0

    def clear_screen(self):
        if self.state != 'idle':
            self.get_logger().warn("Cannot clear while drawing. Send '0' to stop first.")
            return
            
        self.get_logger().info("Clearing screen...")
        
        req = Empty.Request()
        future = self.clear_client.call_async(req)
        
        time.sleep(0.1)
        self.set_pen(off=True)
        self.teleport_client.call_async(self._create_teleport_request(5.5, 5.5, 0.0))

    def _create_teleport_request(self, x, y, theta=0.0):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        return req

    def set_pen(self, r=0, g=0, b=0, width=3, off=False):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = 1 if off else 0
        self.pen_client.call_async(req)
        
    def check_stuck_and_recover(self, distance):
        self.progress_check_timer += 1
        
        if self.progress_check_timer > 50:
            if abs(distance - self.last_distance) < 0.001:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
            
            if self.stuck_counter > 20:
                self.get_logger().warn("Turtle appears stuck, attempting recovery...")
                self.teleport_client.call_async(self._create_teleport_request(self.target_x, self.target_y, 0.0))
                time.sleep(0.1)
                self.stuck_counter = 0
                self.next_point()
            
            self.last_distance = distance
            self.progress_check_timer = 0

    def control_loop(self):
        if self.state != 'moving' or not self.current_pose:
            return

        dx = self.target_x - self.current_pose.x
        dy = self.target_y - self.current_pose.y
        dist = math.sqrt(dx**2 + dy**2)

        self.check_stuck_and_recover(dist)

        twist = Twist()

        if dist < 0.05:
            self.next_point()
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(target_angle - self.current_pose.theta),
                                math.cos(target_angle - self.current_pose.theta))

        if abs(angle_diff) > 0.2:
            twist.angular.z = 3.0 * angle_diff
            twist.linear.x = 0.3
        else:
            twist.angular.z = 1.5 * angle_diff
            twist.linear.x = 1.8

        twist.angular.z = max(min(twist.angular.z, 4.0), -4.0)

        self.cmd_vel_pub.publish(twist)

    def next_point(self):
        self.point_index += 1
        if self.point_index >= len(self.path_points):
            self.finish_drawing()
            return

        if self.path_points[self.point_index] is None:
            if self.pen_down:
                self.set_pen(off=True)
                self.pen_down = False
                time.sleep(0.05)
            
            self.point_index += 1
            if self.point_index >= len(self.path_points):
                self.finish_drawing()
                return
            
            tx, ty = self.path_points[self.point_index]
            self.teleport_client.call_async(self._create_teleport_request(tx, ty, 0.0))
            time.sleep(0.1)
            
            self.set_pen(off=False)
            self.pen_down = True
            time.sleep(0.05)
            
            self.point_index += 1
            if self.point_index >= len(self.path_points):
                self.finish_drawing()
                return
            
            self.target_x, self.target_y = self.path_points[self.point_index]
        else:
            self.target_x, self.target_y = self.path_points[self.point_index]

        self.stuck_counter = 0
        self.progress_check_timer = 0

    def finish_drawing(self):
        self.set_pen(off=True)
        self.pen_down = False
        self.cmd_vel_pub.publish(Twist())
        self.state = 'idle'
        self.get_logger().info("Finished drawing!")

    def stop_drawing(self):
        self.set_pen(off=True)
        self.pen_down = False
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