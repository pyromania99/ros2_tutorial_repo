import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QCheckBox, QPushButton, QMainWindow
from PyQt5.QtCore import Qt, QTimer

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111, projection='3d')
        super(MplCanvas, self).__init__(self.fig)

class FootPositionGUI(QMainWindow):
    def __init__(self, parent=None):
        super(FootPositionGUI, self).__init__(parent)
        self.setWindowTitle("Foot Position Control")
        self.resize(900, 700)
        
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.manual_mode = True
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        mode_layout = QHBoxLayout()
        self.mode_checkbox = QCheckBox("Manual Control")
        self.mode_checkbox.setChecked(self.manual_mode)
        self.mode_checkbox.stateChanged.connect(self.toggle_mode)
        mode_layout.addWidget(self.mode_checkbox)
        
        self.reset_button = QPushButton("Reset Positions")
        self.reset_button.clicked.connect(self.reset_positions)
        mode_layout.addWidget(self.reset_button)
        
        main_layout.addLayout(mode_layout)
        
        slider_layout = QGridLayout()
        
        slider_layout.addWidget(QLabel("Left Foot X Position"), 0, 0)
        self.left_x_slider = QSlider(Qt.Horizontal)
        self.left_x_slider.setMinimum(-70)
        self.left_x_slider.setMaximum(70)
        self.left_x_slider.setValue(int(self.left_x * 100))
        self.left_x_slider.setTickPosition(QSlider.TicksBelow)
        self.left_x_slider.setTickInterval(10)
        self.left_x_slider.valueChanged.connect(self.update_left_x)
        slider_layout.addWidget(self.left_x_slider, 0, 1)
        self.left_x_label = QLabel(f"X: {self.left_x:.2f}")
        slider_layout.addWidget(self.left_x_label, 0, 2)
        
        slider_layout.addWidget(QLabel("Left Foot Y Position"), 1, 0)
        self.left_y_slider = QSlider(Qt.Horizontal)
        self.left_y_slider.setMinimum(5)
        self.left_y_slider.setMaximum(70)
        self.left_y_slider.setValue(int(self.left_y * 100))
        self.left_y_slider.setTickPosition(QSlider.TicksBelow)
        self.left_y_slider.setTickInterval(10)
        self.left_y_slider.valueChanged.connect(self.update_left_y)
        slider_layout.addWidget(self.left_y_slider, 1, 1)
        self.left_y_label = QLabel(f"Y: {self.left_y:.2f}")
        slider_layout.addWidget(self.left_y_label, 1, 2)
        
        slider_layout.addWidget(QLabel("Right Foot X Position"), 2, 0)
        self.right_x_slider = QSlider(Qt.Horizontal)
        self.right_x_slider.setMinimum(-70)
        self.right_x_slider.setMaximum(70)
        self.right_x_slider.setValue(int(self.right_x * 100))
        self.right_x_slider.setTickPosition(QSlider.TicksBelow)
        self.right_x_slider.setTickInterval(10)
        self.right_x_slider.valueChanged.connect(self.update_right_x)
        slider_layout.addWidget(self.right_x_slider, 2, 1)
        self.right_x_label = QLabel(f"X: {self.right_x:.2f}")
        slider_layout.addWidget(self.right_x_label, 2, 2)
        
        slider_layout.addWidget(QLabel("Right Foot Y Position"), 3, 0)
        self.right_y_slider = QSlider(Qt.Horizontal)
        self.right_y_slider.setMinimum(5)
        self.right_y_slider.setMaximum(70)
        self.right_y_slider.setValue(int(self.right_y * 100))
        self.right_y_slider.setTickPosition(QSlider.TicksBelow)
        self.right_y_slider.setTickInterval(10)
        self.right_y_slider.valueChanged.connect(self.update_right_y)
        slider_layout.addWidget(self.right_y_slider, 3, 1)
        self.right_y_label = QLabel(f"Y: {self.right_y:.2f}")
        slider_layout.addWidget(self.right_y_label, 3, 2)
        
        self.status_label = QLabel("Mode: Manual Control")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        
        main_layout.addLayout(slider_layout)
        main_layout.addWidget(self.status_label)
        
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        main_layout.addWidget(self.canvas)
        
        self.left_foot_history = []
        self.right_foot_history = []
        
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)
        
    def update_left_x(self, value):
        self.left_x = value / 100.0
        self.left_x_label.setText(f"X: {self.left_x:.2f}")
        
    def update_left_y(self, value):
        self.left_y = value / 100.0
        self.left_y_label.setText(f"Y: {self.left_y:.2f}")
        
    def update_right_x(self, value):
        self.right_x = value / 100.0
        self.right_x_label.setText(f"X: {self.right_x:.2f}")
        
    def update_right_y(self, value):
        self.right_y = value / 100.0
        self.right_y_label.setText(f"Y: {self.right_y:.2f}")
    
    def toggle_mode(self, state):
        self.manual_mode = (state == Qt.Checked)
        mode_text = "Manual Control" if self.manual_mode else "Trajectory Mode"
        self.status_label.setText(f"Mode: {mode_text}")
        
        enabled = self.manual_mode
        self.left_x_slider.setEnabled(enabled)
        self.left_y_slider.setEnabled(enabled)
        self.right_x_slider.setEnabled(enabled)
        self.right_y_slider.setEnabled(enabled)
    
    def reset_positions(self):
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        
        self.left_x_slider.setValue(int(self.left_x * 100))
        self.left_y_slider.setValue(int(self.left_y * 100))
        self.right_x_slider.setValue(int(self.right_x * 100))
        self.right_y_slider.setValue(int(self.right_y * 100))
        
    def get_positions(self):
        return (self.left_x, self.left_y), (self.right_x, self.right_y)
    
    def get_manual_mode(self):
        return self.manual_mode
    
    def update_plot(self):
        if len(self.left_foot_history) > 1:
            lx, ly, lz = zip(*self.left_foot_history)
            rx, ry, rz = zip(*self.right_foot_history)

            current_time = max(lz[-1], rz[-1])
            min_time = max(0, current_time - 2.5)
            max_time = current_time + 0.5

            self.canvas.axes.clear()
            self.canvas.axes.plot(lz, lx, ly, 'r', label='Left Foot')
            self.canvas.axes.plot(rz, rx, ry, 'b', label='Right Foot')
            
            self.canvas.axes.set_xlim(min_time, max_time)
            self.canvas.axes.set_ylim(-0.4, 0.4)
            self.canvas.axes.set_zlim(0, 1.0)
            
            self.canvas.axes.set_xlabel("Time (s)")
            self.canvas.axes.set_ylabel("X (m)")
            self.canvas.axes.set_zlabel("Y (m)")
            
            mode_str = "Manual Control" if self.manual_mode else "Trajectory Mode"
            self.canvas.axes.set_title(f"3D Foot Trajectory - {mode_str}")
            self.canvas.axes.legend()
            self.canvas.draw()


class GaitIKPublisher(Node):
    def __init__(self, gui):
        super().__init__('gait_ik_publisher')
        self.gui = gui
        
        self.publisher = self.create_publisher(Float64MultiArray, '/target_positions', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/foot_trajectory_markers', 10)
        
        self.step_index = 0
        self.thigh = 0.36
        self.shin = 0.31

        self.gait_trajectory = self.generate_gait_trajectory()
        
        self.left_foot = (0.0, 0.0)
        self.right_foot = (0.0, 0.0)

        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.timer_callback)
        self.control_timer.start(50)
        
        self.marker_timer = QTimer()
        self.marker_timer.timeout.connect(self.publish_trajectory_markers)
        self.marker_timer.start(500)
        
        self.get_logger().info('Gait IK Publisher started.')

    def generate_gait_trajectory(self):
        step_count = 200
        step_length = 0.2
        step_height = 0.05
        support_ratio = 0.6
        
        leg_length = self.thigh + self.shin
        
        support_points = int(step_count * support_ratio)
        swing_points = step_count - support_points
        
        support_x = np.linspace(step_length/2, -step_length/2, support_points)
        support_y = np.zeros(support_points)-leg_length
        
        swing_x = np.linspace(-step_length/2, step_length/2, swing_points)
        swing_y = step_height * np.sin(np.pi * np.linspace(0, 1, swing_points))-leg_length
        
        x = np.concatenate([support_x, swing_x])
        y = np.concatenate([support_y, swing_y])
        
        return list(zip(x, y))

    def inverse_kinematics(self, foot_pos):
        x, y = foot_pos
        L1, L2 = self.thigh, self.shin

        dist = math.sqrt(x**2 + y**2)
        dist = max(min(dist, L1 + L2), 1e-5)
        cos_knee = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
        cos_knee = max(min(cos_knee, 1.0), -1.0)

        interior_knee_angle = math.acos(cos_knee)
        
        bending_forward = True
        if y > 0 and dist < (L1 + L2) * 0.98:
            bending_forward = False
            
        knee_angle = math.pi - interior_knee_angle
        
        if not bending_forward:
            knee_angle = -knee_angle
        
        target_angle = math.atan2(y, x) + math.pi/2
        cos_hip_internal = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
        cos_hip_internal = max(min(cos_hip_internal, 1.0), -1.0)
        hip_internal_angle = math.acos(cos_hip_internal)
        
        hip_angle = target_angle - hip_internal_angle
        
        return [hip_angle, knee_angle]

    def calculate_knee_position(self, foot_pos, angles, leg_side):
        hip_angle, knee_angle = angles
        
        hip_y_offset = 0.1 if leg_side == 'left' else -0.1
        hip_pos = [0.0, hip_y_offset, 0.0]
        
        if leg_side == 'right':
            hip_angle = -hip_angle
        
        knee_x = hip_pos[0] + self.thigh * math.sin(hip_angle)
        knee_y = hip_pos[1]
        knee_z = hip_pos[2] - self.thigh * math.cos(hip_angle)
        
        return knee_x, knee_y, knee_z

    def timer_callback(self):
        manual_mode = self.gui.get_manual_mode()
        
        if manual_mode:
            left_pos, right_pos = self.gui.get_positions()
            self.left_foot = (left_pos[0], left_pos[1] - (self.thigh + self.shin))
            self.right_foot = (right_pos[0], right_pos[1] - (self.thigh + self.shin))
        else:
            num_steps = len(self.gait_trajectory)
            left_idx = self.step_index % num_steps
            right_idx = (self.step_index + num_steps // 2) % num_steps
            
            self.left_foot = self.gait_trajectory[left_idx]
            self.right_foot = self.gait_trajectory[right_idx]
        
        left_angles = self.inverse_kinematics(self.left_foot)
        right_angles = self.inverse_kinematics(self.right_foot)

        if self.step_index % 100 == 0:
            mode_str = "Manual" if manual_mode else "Trajectory"
            self.get_logger().info(f"Mode: {mode_str}")
            self.get_logger().info(f"Left foot position: {self.left_foot}")
            self.get_logger().info(f"Right foot position: {self.right_foot}")

        joint_cmds = Float64MultiArray()
        joint_cmds.data = [
            0.0, left_angles[0], left_angles[1],
            0.0, -right_angles[0], right_angles[1]
        ]
        self.publisher.publish(joint_cmds)

        timestamp = self.step_index * 0.05
        self.gui.left_foot_history.append((self.left_foot[0], self.left_foot[1], timestamp))
        self.gui.right_foot_history.append((self.right_foot[0], self.right_foot[1], timestamp))
        
        if len(self.gui.left_foot_history) > 500:
            self.gui.left_foot_history.pop(0)
            self.gui.right_foot_history.pop(0)

        self.step_index += 1

    def publish_trajectory_markers(self):
        marker_array = MarkerArray()
        
        left_marker = Marker()
        left_marker.header.frame_id = "base_link"
        left_marker.header.stamp = self.get_clock().now().to_msg()
        left_marker.ns = "foot_trajectories"
        left_marker.id = 0
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        left_marker.pose.orientation.w = 1.0
        left_marker.scale.x = 0.02
        left_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        right_marker = Marker()
        right_marker.header.frame_id = "base_link"
        right_marker.header.stamp = self.get_clock().now().to_msg()
        right_marker.ns = "foot_trajectories"
        right_marker.id = 1
        right_marker.type = Marker.LINE_STRIP
        right_marker.action = Marker.ADD
        right_marker.pose.orientation.w = 1.0
        right_marker.scale.x = 0.02
        right_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        
        manual_mode = self.gui.get_manual_mode()
        
        if manual_mode:
            history = self.gui.left_foot_history
            history_length = min(len(history), 100)
            for i in range(max(0, len(history) - history_length), len(history)):
                left_point = Point()
                left_point.x = history[i][0]
                left_point.y = 0.1
                left_point.z = history[i][1]
                left_marker.points.append(left_point)
                
                right_point = Point()
                right_point.x = self.gui.right_foot_history[i][0]
                right_point.y = -0.1
                right_point.z = self.gui.right_foot_history[i][1]
                right_marker.points.append(right_point)
        else:
            num_steps = len(self.gait_trajectory)
            current_idx = self.step_index % num_steps
            
            for i in range(num_steps):
                left_idx = (current_idx + i) % num_steps
                left_pos = self.gait_trajectory[left_idx]
                left_point = Point()
                left_point.x = left_pos[0]
                left_point.y = 0.1
                left_point.z = left_pos[1]
                left_marker.points.append(left_point)
                
                right_idx = (current_idx + i + num_steps // 2) % num_steps
                right_pos = self.gait_trajectory[right_idx]
                right_point = Point()
                right_point.x = right_pos[0]
                right_point.y = -0.1
                right_point.z = right_pos[1]
                right_marker.points.append(right_point)
        
        current_left_marker = Marker()
        current_left_marker.header.frame_id = "base_link"
        current_left_marker.header.stamp = self.get_clock().now().to_msg()
        current_left_marker.ns = "foot_trajectories"
        current_left_marker.id = 2
        current_left_marker.type = Marker.SPHERE
        current_left_marker.action = Marker.ADD
        current_left_marker.pose.orientation.w = 1.0
        current_left_marker.scale.x = 0.05
        current_left_marker.scale.y = 0.05
        current_left_marker.scale.z = 0.05
        current_left_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        
        current_left_marker.pose.position.x = self.left_foot[0]
        current_left_marker.pose.position.y = 0.1
        current_left_marker.pose.position.z = self.left_foot[1]
        
        current_right_marker = Marker()
        current_right_marker.header.frame_id = "base_link"
        current_right_marker.header.stamp = self.get_clock().now().to_msg()
        current_right_marker.ns = "foot_trajectories"
        current_right_marker.id = 3
        current_right_marker.type = Marker.SPHERE
        current_right_marker.action = Marker.ADD
        current_right_marker.pose.orientation.w = 1.0
        current_right_marker.scale.x = 0.05
        current_right_marker.scale.y = 0.05
        current_right_marker.scale.z = 0.05
        current_right_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        
        current_right_marker.pose.position.x = self.right_foot[0]
        current_right_marker.pose.position.y = -0.1
        current_right_marker.pose.position.z = self.right_foot[1]
        
        left_knee_marker = Marker()
        left_knee_marker.header.frame_id = "base_link"
        left_knee_marker.header.stamp = self.get_clock().now().to_msg()
        left_knee_marker.ns = "joint_markers"
        left_knee_marker.id = 4
        left_knee_marker.type = Marker.SPHERE
        left_knee_marker.action = Marker.ADD
        left_knee_marker.pose.orientation.w = 1.0
        left_knee_marker.scale.x = 0.04
        left_knee_marker.scale.y = 0.04
        left_knee_marker.scale.z = 0.04
        left_knee_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        
        right_knee_marker = Marker()
        right_knee_marker.header.frame_id = "base_link"
        right_knee_marker.header.stamp = self.get_clock().now().to_msg()
        right_knee_marker.ns = "joint_markers"
        right_knee_marker.id = 5
        right_knee_marker.type = Marker.SPHERE
        right_knee_marker.action = Marker.ADD
        right_knee_marker.pose.orientation.w = 1.0
        right_knee_marker.scale.x = 0.04
        right_knee_marker.scale.y = 0.04
        right_knee_marker.scale.z = 0.04
        right_knee_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        
        left_angles = self.inverse_kinematics(self.left_foot)
        right_angles = self.inverse_kinematics(self.right_foot)
        
        left_knee_pos = self.calculate_knee_position(self.left_foot, left_angles, 'left')
        right_knee_pos = self.calculate_knee_position(self.right_foot, right_angles, 'right')
        
        left_knee_marker.pose.position.x = left_knee_pos[0]
        left_knee_marker.pose.position.y = left_knee_pos[1]
        left_knee_marker.pose.position.z = left_knee_pos[2]
        
        right_knee_marker.pose.position.x = right_knee_pos[0]
        right_knee_marker.pose.position.y = right_knee_pos[1]
        right_knee_marker.pose.position.z = right_knee_pos[2]
        
        left_leg_marker = Marker()
        left_leg_marker.header.frame_id = "base_link"
        left_leg_marker.header.stamp = self.get_clock().now().to_msg()
        left_leg_marker.ns = "leg_links"
        left_leg_marker.id = 6
        left_leg_marker.type = Marker.LINE_LIST
        left_leg_marker.action = Marker.ADD
        left_leg_marker.pose.orientation.w = 1.0
        left_leg_marker.scale.x = 0.02
        left_leg_marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)
        
        hip_point = Point(x=0.0, y=0.1, z=0.0)
        knee_point = Point(x=left_knee_pos[0], y=left_knee_pos[1], z=left_knee_pos[2])
        foot_point = Point(x=self.left_foot[0], y=0.1, z=self.left_foot[1])
        
        left_leg_marker.points.append(hip_point)
        left_leg_marker.points.append(knee_point)
        left_leg_marker.points.append(knee_point)
        left_leg_marker.points.append(foot_point)
        
        right_leg_marker = Marker()
        right_leg_marker.header.frame_id = "base_link"
        right_leg_marker.header.stamp = self.get_clock().now().to_msg()
        right_leg_marker.ns = "leg_links"
        right_leg_marker.id = 7
        right_leg_marker.type = Marker.LINE_LIST
        right_leg_marker.action = Marker.ADD
        right_leg_marker.pose.orientation.w = 1.0
        right_leg_marker.scale.x = 0.02
        right_leg_marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)
        
        hip_point = Point(x=0.0, y=-0.1, z=0.0)
        knee_point = Point(x=right_knee_pos[0], y=right_knee_pos[1], z=right_knee_pos[2])
        foot_point = Point(x=self.right_foot[0], y=-0.1, z=self.right_foot[1])
        
        right_leg_marker.points.append(hip_point)
        right_leg_marker.points.append(knee_point)
        right_leg_marker.points.append(knee_point)
        right_leg_marker.points.append(foot_point)
        
        marker_array.markers.append(left_marker)
        marker_array.markers.append(right_marker)
        marker_array.markers.append(current_left_marker)
        marker_array.markers.append(current_right_marker)
        marker_array.markers.append(left_knee_marker)
        marker_array.markers.append(right_knee_marker)
        marker_array.markers.append(left_leg_marker)
        marker_array.markers.append(right_leg_marker)
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = FootPositionGUI()
    gui.show()
    node = GaitIKPublisher(gui)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    ros_timer = QTimer()
    def process_ros():
        executor.spin_once(timeout_sec=0)
    ros_timer.timeout.connect(process_ros)
    ros_timer.start(10)
    
    exit_code = app.exec_()
    
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()