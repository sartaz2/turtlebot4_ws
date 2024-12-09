import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.waypoints = [
            (1.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.0, 1.0)
        ]  # Predefined waypoints
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.lidar_ranges = []  # Store Lidar readings
        self.min_distance_threshold = 0.5  # Minimum safe distance from an obstacle

        self.timer = self.create_timer(0.1, self.navigate)

    def lidar_callback(self, msg):
        """Callback function for Lidar data."""
        self.lidar_ranges = msg.ranges
        self.obstacle_detected = any(
            dist < self.min_distance_threshold for dist in self.lidar_ranges if dist > 0.0
        )

    def navigate(self):
        """Main navigation logic combining waypoint navigation and obstacle avoidance."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping the robot.')
            self.stop_robot()
            return

        if self.obstacle_detected:
            self.get_logger().info('Obstacle detected! Avoiding...')
            self.avoid_obstacle()
        else:
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}')
            self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        """Navigate to the current waypoint."""
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        twist = Twist()

        # Calculate distance and angle to the waypoint
        robot_x, robot_y, robot_yaw = self.get_robot_pose()
        distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        angle_diff = angle_to_target - robot_yaw

        # Adjust robot heading towards the waypoint
        if abs(angle_diff) > 0.1:
            twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        else:
            twist.linear.x = self.linear_speed

        # Publish velocity command
        self.cmd_vel_publisher.publish(twist)

        # Check if the waypoint is reached
        if distance < 0.2:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached.')
            self.current_waypoint_index += 1

    def avoid_obstacle(self):
        """Avoid obstacles by moving towards free space."""
        twist = Twist()

        # Stop forward motion if obstacle detected
        twist.linear.x = 0.0

        # Identify the direction with the maximum distance
        if self.lidar_ranges:
            # Find the index of the maximum range
            max_range_index = max(
                range(len(self.lidar_ranges)),
                key=lambda i: self.lidar_ranges[i] if self.lidar_ranges[i] > 0.0 else -1
            )
            max_distance = self.lidar_ranges[max_range_index]

            # Convert index to an angle
            angle_to_turn = (max_range_index * math.radians(360 / len(self.lidar_ranges))) - math.pi

            # Set velocity to turn towards free space and move if safe
            if max_distance > self.min_distance_threshold:
                twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * (-1 if angle_to_turn < 0 else 1)
        else:
            # Default to rotating if no Lidar data
            twist.angular.z = self.angular_speed

        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def get_robot_pose(self):
        """Stub for getting the robot's current pose.
        Replace this with odometry or localization integration."""
        # Assuming the robot starts at (0, 0) facing 0 radians
        return 0.0, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

