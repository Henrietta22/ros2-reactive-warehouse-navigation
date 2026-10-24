#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # LiDAR subscriber (best-effort QoS compatible with Gazebo scan topic)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Thresholds (in meters)
        self.safe_distance = 0.65
        self.critical_distance = 0.35

        self.get_logger().info('Obstacle Avoidance Node Initialized and Running.')

    def scan_callback(self, msg: LaserScan):
        # Filter valid scan ranges (ignore inf, nan, and 0.0)
        def clean_range(ranges):
            valid = [r for r in ranges if not math.isnan(r) and not math.isinf(r) and r > msg.range_min]
            return min(valid) if valid else msg.range_max

        total_rays = len(msg.ranges)
        if total_rays == 0:
            return

        # Sector indexing (360-degree LiDAR):
        # Front: 0 to 30 deg and 330 to 360 deg
        # Left: 30 to 90 deg
        # Right: 270 to 330 deg
        front_left = msg.ranges[0:30]
        front_right = msg.ranges[-30:]
        front_dist = clean_range(front_left + front_right)

        left_dist = clean_range(msg.ranges[30:90])
        right_dist = clean_range(msg.ranges[270:330])

        twist = Twist()

        # Decision State Machine
        if front_dist < self.critical_distance:
            # Emergency fallback: reverse and pivot
            self.get_logger().warn(f'Too close ({front_dist:.2f}m)! Backing up...')
            twist.linear.x = -0.05
            twist.angular.z = 0.5
        elif front_dist < self.safe_distance:
            # Obstacle ahead: steer toward the side with more clearance
            twist.linear.x = 0.05
            if left_dist > right_dist:
                self.get_logger().info(f'Turning Left (Clearance: L={left_dist:.2f}m, R={right_dist:.2f}m)')
                twist.angular.z = 0.5
            else:
                self.get_logger().info(f'Turning Right (Clearance: L={left_dist:.2f}m, R={right_dist:.2f}m)')
                twist.angular.z = -0.5
        else:
            # Clear corridor: cruise forward
            twist.linear.x = 0.22
            # Proportional centering: steer slightly away from walls on either flank
            if left_dist < 0.4:
                twist.angular.z = -0.2
            elif right_dist < 0.4:
                twist.angular.z = 0.2
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot when exiting
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
