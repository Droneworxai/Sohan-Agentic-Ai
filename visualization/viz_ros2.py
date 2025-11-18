#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import json
import threading

class RobotVisualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        
        self.x = 0.0
        self.y = 0.0
        self.path_x = []
        self.path_y = []
        
        self.subscription = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10
        )
        
        self.get_logger().info('Visualizer ready')
    
    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.path_x.append(self.x)
        self.path_y.append(self.y)
        
        if len(self.path_x) % 10 == 0:
            self.get_logger().info(f'Robot at: ({self.x:.1f}, {self.y:.1f})')

def ros_spin(node):
    rclpy.spin(node)

rclpy.init()
viz = RobotVisualizer()

# Start ROS in background
thread = threading.Thread(target=ros_spin, args=(viz,), daemon=True)
thread.start()

# Load waypoints
with open('/home/droneworx/working/mission_plan.json', 'r') as f:
    plan = json.load(f)
    waypoints = [(x, y) for x, y in plan['local_coords']]
    print(f"✓ Loaded {len(waypoints)} waypoints")

# Setup plot
plt.ion()
fig, ax = plt.subplots(figsize=(12, 8))

wx, wy = zip(*waypoints)
ax.plot(wx, wy, 'bo', markersize=8, alpha=0.6, label='Waypoints')

path_line, = ax.plot([], [], 'g-', linewidth=2, label='Path')
robot_dot, = ax.plot([], [], 'ro', markersize=20, label='Robot')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Mission Planning')
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')

if waypoints:
    wx, wy = zip(*waypoints)
    x_margin = (max(wx) - min(wx)) * 0.1
    y_margin = (max(wy) - min(wy)) * 0.1
    ax.set_xlim(min(wx) - x_margin, max(wx) + x_margin)
    ax.set_ylim(min(wy) - y_margin, max(wy) + y_margin)

print("Visualization running")

try:
    while True:
        if len(viz.path_x) > 0:
            path_line.set_data(viz.path_x, viz.path_y)
            robot_dot.set_data([viz.x], [viz.y])
            ax.relim()
            ax.autoscale_view()
        
        plt.pause(0.2)
        
except KeyboardInterrupt:
    print("Stopped")

viz.destroy_node()
rclpy.shutdown()