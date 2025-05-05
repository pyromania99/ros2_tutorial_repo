import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread
import time
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class GaitIKPublisher(Node):
    def __init__(self):
        super().__init__('gait_ik_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, '/target_positions', 10)
        
        # Add publishers for visualization markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/foot_trajectory_markers', 10)

        self.gait_trajectory = self.generate_gait_trajectory()

        self.thigh = 0.315
        self.shin = 0.329

        self.timer_period = 0.05  # 20 Hz
        self.step_index = 0

        # Lists to store foot positions for 3D plotting
        self.left_foot_history = []
        self.right_foot_history=[]
        
        # Base height of the robot's hip joint from ground
        self.hip_height = 0.6  # Adjust this based on your robot's dimensions

        self.create_timer(self.timer_period, self.timer_callback)
        
        # Timer for publishing visualization markers
        self.create_timer(0.5, self.publish_trajectory_markers)  # 2Hz is enough for visualization

        # Start 3D plotting in a separate thread
        self.plot_thread = Thread(target=self.plot_3d_trajectory)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def generate_gait_trajectory(self):
        step_count = 50
        step_length = 0.2
        step_height = 0.05

        x = np.linspace(-step_length / 2, step_length / 2, step_count)
        y = step_height * np.sin(np.pi * np.linspace(0, 1, step_count))
        return list(zip(x, y))

    def inverse_kinematics(self, foot_pos):
        x, y = foot_pos
        L1, L2 = self.thigh, self.shin
        dist = math.sqrt(x**2 + y**2)
        dist = min(dist, L1 + L2 - 1e-5)

        cos_knee = (dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
        knee_angle = math.acos(cos_knee)

        angle_to_foot = math.atan2(y, x)
        cos_hip = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
        hip_offset = math.acos(cos_hip)
        hip_angle = angle_to_foot - hip_offset

        return [hip_angle, -knee_angle]

    def timer_callback(self):
        n = len(self.gait_trajectory)
        left_index = self.step_index % n
        right_index = (self.step_index + n // 2) % n

        left_foot = self.gait_trajectory[left_index]
        right_foot = self.gait_trajectory[right_index]

        left_angles = self.inverse_kinematics(left_foot)
        right_angles = self.inverse_kinematics(right_foot)

        self.get_logger().info(f"Left foot spline position: {left_foot}")
        self.get_logger().info(f"Right foot spline position: {right_foot}")
        self.get_logger().info(f"Left joint angles: {left_angles}")
        self.get_logger().info(f"Right joint angles: {right_angles}")

        joint_cmds = Float64MultiArray()
        joint_cmds.data = [
            0.0,left_angles[0], left_angles[1],
            0.0,right_angles[0], right_angles[1]
        ]
        self.publisher.publish(joint_cmds)
        self.get_logger().info(f"Published joint commands: {joint_cmds.data}")

        # Store foot positions (x, y, time as z)
        timestamp = self.step_index * self.timer_period
        self.left_foot_history.append((left_foot[0], left_foot[1], timestamp))
        self.right_foot_history.append((right_foot[0], right_foot[1], timestamp))

        self.step_index += 1

    def plot_3d_trajectory(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D Foot Trajectories")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("X (m)")
        ax.set_zlabel("Y (m)")

        while True:
            if len(self.left_foot_history) > 1:
                lx, ly, lz = zip(*self.left_foot_history)
                rx, ry, rz = zip(*self.right_foot_history)

                # Get current time for setting limits
                current_time = max(lz[-1], rz[-1])
                min_time = max(0, current_time - 2.5)
                max_time = current_time + 2.5

                ax.clear()
                # Swap axes: lz (time) as x, lx as y, ly as z
                ax.plot3D(lz, lx, ly, 'r', label='Left Foot')
                ax.plot3D(rz, rx, ry, 'b', label='Right Foot')
                
                # Set new limits
                ax.set_xlim(min_time, max_time)
                ax.set_ylim(-0.15, 0.15)
                ax.set_zlim(-0.1, 0.1)
                
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("X (m)")
                ax.set_zlabel("Y (m)")
                ax.set_title("3D Foot Trajectory Over Time")
                ax.legend()
                plt.pause(0.05)
            time.sleep(0.1)

    def publish_trajectory_markers(self):
        marker_array = MarkerArray()
        
        # Create a marker for the left foot trajectory
        left_marker = Marker()
        left_marker.header.frame_id = "base_link"  # Use your robot's base frame
        left_marker.header.stamp = self.get_clock().now().to_msg()
        left_marker.ns = "foot_trajectories"
        left_marker.id = 0
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        left_marker.pose.orientation.w = 1.0
        left_marker.scale.x = 0.02  # Line width
        left_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        
        # Create a marker for the right foot trajectory