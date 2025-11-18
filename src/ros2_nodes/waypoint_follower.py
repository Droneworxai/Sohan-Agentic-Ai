#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
import json
import os

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Load waypoints from mission plan
        self.waypoints = self.load_mission_plan()
        
        if not self.waypoints:
            self.get_logger().error('❌ No waypoints loaded!')
            return
        
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 3.0
        
        # Control parameters
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.k_linear = 0.5
        self.k_angular = 1.5
        
        self.navigation_complete = False
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10
        )
        
        # Control loop (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Waypoint Follower initialized')
        self.get_logger().info(f'   Loaded {len(self.waypoints)} waypoints')
    
    def load_mission_plan(self):
        """Load waypoints from mission_plan.json"""
        
        # Try multiple locations
        possible_paths = [
            os.path.expanduser('~/working/mission_plan.json'),
            os.path.expanduser('~/mission_plan.json'),
            'mission_plan.json'
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        plan = json.load(f)
                    
                    self.get_logger().info(f'    Loaded plan from: {path}')
                    
                    if 'strategy' in plan:
                        self.get_logger().info(f'    Strategy: {plan["strategy"]}')
                    
                    if 'lane_order' in plan:
                        self.get_logger().info(f'     Lane order: {plan["lane_order"]}')
                    
                    # Extract waypoints as tuples
                    coords = plan.get('local_coords', [])
                    waypoints = [(x, y) for x, y in coords]
                    
                    return waypoints
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to load {path}: {e}')
        
        # Fallback to demo waypoints
        self.get_logger().warn('  mission_plan.json not found, using demo waypoints')
        return [
            (0.0, 0.0), (10.0, 0.0), (20.0, 0.0)
        ]
    
    def pose_callback(self, msg):
        """Update current position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)
    
    def control_loop(self):
        """Main navigation control"""
        if self.navigation_complete:
            self.publish_velocity(0.0, 0.0)
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info(' All waypoints reached!')
            self.navigation_complete = True
            return
        
        # Get target
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if reached
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f'✓ Waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} reached'
            )
            self.publish_velocity(0.0, 0.0)
            self.current_waypoint_idx += 1
            return
        
        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)
        angular_error = desired_theta - self.current_theta
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))
        
        # Proportional control
        linear_vel = self.k_linear * distance
        angular_vel = self.k_angular * angular_error
        
        # Apply limits
        linear_vel = max(min(linear_vel, self.max_linear_speed), 0.0)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Slow down if turning
        if abs(angular_error) > 0.3:
            linear_vel *= 0.3
        
        self.publish_velocity(linear_vel, angular_vel)
    
    def publish_velocity(self, linear, angular):
        """Send velocity command"""
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
