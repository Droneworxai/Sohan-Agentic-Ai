#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.dt = 0.05
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(self.dt, self.update)
        
        self.get_logger().info('Robot Simulator READY')
    
    def cmd_vel_callback(self, msg):
        self.velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
    
    def update(self):
        self.x += self.velocity * math.cos(self.theta) * self.dt
        self.y += self.velocity * math.sin(self.theta) * self.dt
        self.theta += self.angular_velocity * self.dt
        self.publish_pose()
    
    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = math.sin(self.theta / 2)
        msg.pose.orientation.w = math.cos(self.theta / 2)
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()