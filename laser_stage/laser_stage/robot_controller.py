import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug1Algorithm(Node):

    def __init__(self):
        super().__init__('bug1_algorithm')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher for velocity commands
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Subscription to odometry data
        self.subscription_laser = self.create_subscription(LaserScan, '/base_scan', self.laser_callback, 10)  # Subscription to laser scan data
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer for periodic callback
        
        self.current_position = (0.0, 0.0)  # Current position of the robot
        self.current_orientation = (0.0, 0.0, 0.0, 1.0)  # Current orientation of the robot
        self.laser_data = []  # List to store laser scan data
        self.twist = 1  # Direction multiplier for obstacle avoidance
        self.velocity = 4  # Velocity multiplier
        
        self.goal_positions = [(11.5, 11.5), (10.5, 2.5)]  # List of goal positions
        self.goal_index = 0  # Index of the current goal
        self.goal_position = self.goal_positions[self.goal_index]  # Current goal position
        self.state = 'GO_TO_GOAL'  # Initial state
        self.borde_time = 0  # Time of the last border detection

        self.get_logger().info("Bug1 Algorithm Node has been started")  # Log message

    def timer_callback(self):
        twist = Twist()
        # Calculate the differences between the current position and the goal position
        differences = [abs(c - g) for c, g in zip(self.current_position, self.goal_position)]
        
        # Check if the maximum difference is less than 0.1
        if max(differences) < 0.1:
            if self.goal_index < len(self.goal_positions) - 1:
                self.goal_index += 1
                self.goal_position = self.goal_positions[self.goal_index]
                self.state = 'GO_TO_GOAL'
                self.get_logger().info(f'Goal {self.goal_index} reached. Moving to next goal: {self.goal_position}')
            else:
                self.state = 'STOP'
                self.get_logger().info("All goals reached, stopping")

        if self.state == 'GO_TO_GOAL':
            if self.is_obstacle_around(0.3):  # Check for obstacles within 0.3 meters
                self.state = 'FOLLOW_OBSTACLE'
                self.get_logger().info("Obstacle detected, switching to FOLLOW_OBSTACLE")
            else:
                twist = self.calculate_navigation_twist(self.goal_position)  # Navigate towards the goal

        elif self.state == 'FOLLOW_OBSTACLE':
            if self.is_obstacle_in_front(0.3):  # Check for obstacles directly in front within 0.3 meters
                twist = self.follow_obstacle_twist()  # Follow the obstacle
            elif self.is_obstacle_around(0.5):  # Check for obstacles within 0.5 meters
                twist = self.follow_obstacle_direct()  # Move directly forward to avoid obstacle
            else:
                self.state = 'GO_TO_GOAL'
                self.get_logger().info("Obstacle no longer in front, switching to GO_TO_GOAL")

            # Check if we are at the border and if at least 10 seconds have passed since the last detection
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if self.esta_en_el_borde() and (current_time - self.borde_time > 60):
                self.state = 'GO_TO_GOAL'
                self.twist *= -1  # Change direction
                self.borde_time = current_time  # Update the time of the last border detection
                self.get_logger().info("We are on the border")

        if self.state != 'STOP':
            self.publisher_.publish(twist)  # Publish the twist message
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)  # Stop the robot
            self.get_logger().info("Goal reached, congrats :)")

    def odom_callback(self, msg):
        # Update current position and orientation from odometry data
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_orientation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    def laser_callback(self, msg):
        self.laser_data = msg.ranges  # Update laser scan data

    def is_obstacle_around(self, umbral = 0.3):
        if not self.laser_data:
            return False
        return min(self.laser_data) < umbral  # Check for obstacles within the specified threshold

    def is_obstacle_in_front(self, umbral = 0.3):
        if not self.laser_data:
            return False
        front_distances = self.laser_data[135-60:135+60]  # 120 degrees in front of the robot
        return min(front_distances) < umbral  # Check for obstacles directly in front within the specified threshold

    def follow_obstacle_twist(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.twist * 0.3 * self.velocity  # Turn left or right to follow the obstacle
        return twist
    
    def follow_obstacle_direct(self):
        twist = Twist()
        twist.linear.x = 0.2 * self.velocity
        twist.angular.z = 0.0  # Move straight forward to get past the obstacle
        return twist

    def calculate_navigation_twist(self, target):
        twist = Twist()
        angle_to_target = math.atan2(target[1] - self.current_position[1], target[0] - self.current_position[0])
        yaw = self.get_yaw()
        angle_difference = self.normalize_angle(angle_to_target - yaw)

        if abs(angle_difference) > 0.1:
            twist.angular.z = 0.3 * math.copysign(1, angle_difference) * self.velocity  # Turn towards the target
        else:
            twist.linear.x = 0.2 * self.velocity  # Move towards the target
        
        return twist

    def get_yaw(self):
        # Convert quaternion to yaw angle
        x, y, z, w = self.current_orientation
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Normalize the angle to the range [-pi, pi]
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def distance(self, point1, point2):
        # Calculate the Euclidean distance between two points
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    
    def esta_en_el_borde(self):
        # Check if the robot is at the border of the defined map limits
        x = self.current_position[0]
        y = self.current_position[1]
        x_min, x_max = -2, 15
        y_min, y_max = -2, 15
        return (x < x_min or x > x_max) or (y < y_min or y > y_max)

def main(args=None):
    rclpy.init(args=args)  # Initialize the rclpy library
    node = Bug1Algorithm()  # Create an instance of the Bug1Algorithm node
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()  # Destroy the node when done
    rclpy.shutdown()  # Shutdown the rclpy library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed directly