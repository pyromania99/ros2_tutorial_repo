import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread
import time

class GaitIKPublisher(Node):
    def __init__(self):
        super().__init__('gait_ik_publisher')

        self.publisher = self.create_publisher(Float64MultiArray, '/target_positions', 10)

        self.gait_trajectory = self.generate_gait_trajectory()

        self.thigh = 0.15
        self.shin = 0.15

        self.timer_period = 0.05  # 20 Hz
        self.step_index = 0

        # Lists to store foot positions for 3D plotting
        self.left_foot_history = []
        self.right_foot_history = []

        self.create_timer(self.timer_period, self.timer_callback)

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
            left_angles[0], 0.0, left_angles[1],
            right_angles[0], 0.0, right_angles[1]
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
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Time")

        while True:
            if len(self.left_foot_history) > 1:
                lx, ly, lz = zip(*self.left_foot_history)
                rx, ry, rz = zip(*self.right_foot_history)

                ax.clear()
                ax.plot3D(lx, ly, lz, 'r', label='Left Foot')
                ax.plot3D(rx, ry, rz, 'b', label='Right Foot')
                ax.set_xlim(-0.15, 0.15)
                ax.set_ylim(-0.1, 0.1)
                ax.set_zlim(0, 5)
                ax.set_xlabel("X (m)")
                ax.set_ylabel("Y (m)")
                ax.set_zlabel("Time (s)")
                ax.set_title("3D Foot Trajectory Over Time")
                ax.legend()
                plt.pause(0.05)
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = GaitIKPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
