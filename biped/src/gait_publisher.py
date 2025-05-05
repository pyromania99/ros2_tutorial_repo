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

# Matplotlib integration with PyQt5
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
        
        # Initialize position values
        self.left_x = 0.0
        self.left_y = 0.5
        self.right_x = 0.0
        self.right_y = 0.5
        self.manual_mode = True  # Start in manual mode
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        
        # Mode selection
        mode_layout = QHBoxLayout()
        self.mode_checkbox = QCheckBox("Manual Control")
        self.mode_checkbox.setChecked(self.manual_mode)
        self.mode_checkbox.stateChanged.connect(self.toggle_mode)
        mode_layout.addWidget(self.mode_checkbox)
        
        # Reset button
        self.reset_button = QPushButton("Reset Positions")
        self.reset_button.clicked.connect(self.reset_positions)
        mode_layout.addWidget(self.reset_button)
        
        main_layout.addLayout(mode_layout)
        
        # Create slider layout
        slider_layout = QGridLayout()
        
        # Create sliders for left foot
        slider_layout.addWidget(QLabel("Left Foot X Position"), 0, 0)
        self.left_x_slider = QSlider(Qt.Horizontal)
        self.left_x_slider.setMinimum(-40)
        self.left_x_slider.setMaximum(40)
        self.left_x_slider.setValue(int(self.left_x * 100))
        self.left_x_slider.setTickPosition(QSlider.TicksBelow)
        self.left_x_slider.setTickInterval(10)
        self.left_x_slider.valueChanged.connect(self.update_left_x)
        slider_layout.addWidget(self.left_x_slider, 0, 1)
        self.left_x_label = QLabel(f"X: {self.left_x:.2f}")
        slider_layout.addWidget(self.left_x_label, 0, 2)
        
        # ...add other sliders similarly...
        slider_layout.addWidget(QLabel("Left Foot Y Position"), 1, 0)
        self.left_y_slider = QSlider(Qt.Horizontal)
        self.left_y_slider.setMinimum(0)
        self.left_y_slider.setMaximum(100)
        self.left_y_slider.setValue(int(self.left_y * 100))
        self.left_y_slider.setTickPosition(QSlider.TicksBelow)
        self.left_y_slider.setTickInterval(10)
        self.left_y_slider.valueChanged.connect(self.update_left_y)
        slider_layout.addWidget(self.left_y_slider, 1, 1)
        self.left_y_label = QLabel(f"Y: {self.left_y:.2f}")
        slider_layout.addWidget(self.left_y_label, 1, 2)
        
        slider_layout.addWidget(QLabel("Right Foot X Position"), 2, 0)
        self.right_x_slider = QSlider(Qt.Horizontal)
        self.right_x_slider.setMinimum(-40)
        self.right_x_slider.setMaximum(40)
        self.right_x_slider.setValue(int(self.right_x * 100))
        self.right_x_slider.setTickPosition(QSlider.TicksBelow)
        self.right_x_slider.setTickInterval(10)
        self.right_x_slider.valueChanged.connect(self.update_right_x)
        slider_layout.addWidget(self.right_x_slider, 2, 1)
        self.right_x_label = QLabel(f"X: {self.right_x:.2f}")
        slider_layout.addWidget(self.right_x_label, 2, 2)
        
        slider_layout.addWidget(QLabel("Right Foot Y Position"), 3, 0)
        self.right_y_slider = QSlider(Qt.Horizontal)
        self.right_y_slider.setMinimum(0)
        self.right_y_slider.setMaximum(100)
        self.right_y_slider.setValue(int(self.right_y * 100))
        self.right_y_slider.setTickPosition(QSlider.TicksBelow)
        self.right_y_slider.setTickInterval(10)
        self.right_y_slider.valueChanged.connect(self.update_right_y)
        slider_layout.addWidget(self.right_y_slider, 3, 1)
        self.right_y_label = QLabel(f"Y: {self.right_y:.2f}")
        slider_layout.addWidget(self.right_y_label, 3, 2)
        
        # Add status label
        self.status_label = QLabel("Mode: Manual Control")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-weight: bold; color: blue;")
        
        main_layout.addLayout(slider_layout)
        main_layout.addWidget(self.status_label)
        
        # Add Matplotlib canvas to layout
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        main_layout.addWidget(self.canvas)
        
        # Initialize trajectory history
        self.left_foot_history = []
        self.right_foot_history = []
        
        # Setup plot refresh timer
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)  # Refresh plot every 100ms
        
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
        
        # Update slider enabled state
        enabled = self.manual_mode
        self.left_x_slider.setEnabled(enabled)
        self.left_y_slider.setEnabled(enabled)
        self.right_x_slider.setEnabled(enabled)
        self.right_y_slider.setEnabled(enabled)
    
    def reset_positions(self):
        # Reset to default positions
        self.left_x = 0.0
        self.left_y = 0.5
        self.right_x = 0.0
        self.right_y = 0.5
        
        # Update sliders
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
            
            # Adjust limits based on what's being shown
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

        # Store the GUI reference
        self.gui = gui
        
        # Publishers
        self.publisher = self.create_publisher(Float64MultiArray, '/target_positions', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/foot_trajectory_markers', 10)
        
        # Initialize required variables
        self.step_index = 0
        self.gait_trajectory = self.generate_gait_trajectory()
        self.hip_height = 0.6  # Height from ground to hip

        self.thigh = 0.315
        self.shin = 0.329
        
        # Set default foot positions
        self.left_foot = (0.0, 0.5)
        self.right_foot = (0.0, 0.5)

        # Create Qt timers instead of ROS timers
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.timer_callback)
        self.control_timer.start(50)  # 50ms = 20Hz
        
        self.marker_timer = QTimer()
        self.marker_timer.timeout.connect(self.publish_trajectory_markers)
        self.marker_timer.start(500)  # 500ms = 2Hz
        
        self.get_logger().info('Gait IK Publisher started.')

    def generate_gait_trajectory(self):
        step_count = 200
        step_length = 0.2
        step_height = 0.05
        support_points = 20
        
        swing_count = step_count - support_points
        swing_x = np.linspace(-step_length / 2, step_length / 2, swing_count)
        swing_y = step_height * np.sin(np.pi * np.linspace(0, 1, swing_count))
        
        support_x = np.full(support_points, -step_length / 2)
        support_y = np.zeros(support_points)
        
        x = np.concatenate([swing_x, support_x])
        y = np.concatenate([swing_y, support_y])
        
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
        manual_mode = self.gui.get_manual_mode()
        
        if manual_mode:
            # Get position values from GUI sliders
            self.left_foot, self.right_foot = self.gui.get_positions()
        else:
            # Use trajectory mode
            num_steps = len(self.gait_trajectory)
            left_idx = self.step_index % num_steps
            right_idx = (self.step_index + num_steps // 2) % num_steps
            
            self.left_foot = self.gait_trajectory[left_idx]
            self.right_foot = self.gait_trajectory[right_idx]
        
        # Compute joint angles for both legs using inverse kinematics
        left_angles = self.inverse_kinematics(self.left_foot)
        right_angles = self.inverse_kinematics(self.right_foot)

        # Log info at a reduced rate
        if self.step_index % 20 == 0:
            mode_str = "Manual" if manual_mode else "Trajectory"
            self.get_logger().info(f"Mode: {mode_str}")
            self.get_logger().info(f"Left foot position: {self.left_foot}")
            self.get_logger().info(f"Right foot position: {self.right_foot}")

        # Publish joint commands
        joint_cmds = Float64MultiArray()
        joint_cmds.data = [
            0.0, left_angles[0], left_angles[1],
            0.0, right_angles[0], right_angles[1]
        ]
        self.publisher.publish(joint_cmds)

        # Store foot positions for trajectory visualization
        timestamp = self.step_index * 0.05  # timer_period
        self.gui.left_foot_history.append((self.left_foot[0], self.left_foot[1], timestamp))
        self.gui.right_foot_history.append((self.right_foot[0], self.right_foot[1], timestamp))
        
        # Limit history size to avoid memory issues
        if len(self.gui.left_foot_history) > 500:
            self.gui.left_foot_history.pop(0)
            self.gui.right_foot_history.pop(0)

        self.step_index += 1

    def publish_trajectory_markers(self):
        marker_array = MarkerArray()
        
        # Create a marker for the left foot trajectory
        left_marker = Marker()
        left_marker.header.frame_id = "base_link"
        left_marker.header.stamp = self.get_clock().now().to_msg()
        left_marker.ns = "foot_trajectories"
        left_marker.id = 0
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        left_marker.pose.orientation.w = 1.0
        left_marker.scale.x = 0.02  # Line width
        left_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
        
        # Create a marker for the right foot trajectory
        right_marker = Marker()
        right_marker.header.frame_id = "base_link"
        right_marker.header.stamp = self.get_clock().now().to_msg()
        right_marker.ns = "foot_trajectories"
        right_marker.id = 1
        right_marker.type = Marker.LINE_STRIP
        right_marker.action = Marker.ADD
        right_marker.pose.orientation.w = 1.0
        right_marker.scale.x = 0.02  # Line width
        right_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue
        
        # Create marker contents based on current mode
        manual_mode = self.gui.get_manual_mode()
        
        if manual_mode:
            # In manual mode, visualize recent history
            history = self.gui.left_foot_history
            history_length = min(len(history), 100)
            for i in range(max(0, len(history) - history_length), len(history)):
                # Left foot history
                left_point = Point()
                left_point.x = history[i][0]
                left_point.y = 0.1  # Lateral offset for left foot
                left_point.z = self.hip_height - history[i][1]
                left_marker.points.append(left_point)
                
                # Right foot history
                right_point = Point()
                right_point.x = self.gui.right_foot_history[i][0]
                right_point.y = -0.1  # Lateral offset for right foot
                right_point.z = self.hip_height - self.gui.right_foot_history[i][1]
                right_marker.points.append(right_point)
        else:
            # In trajectory mode, visualize the full gait trajectory
            num_steps = len(self.gait_trajectory)
            current_idx = self.step_index % num_steps
            
            for i in range(num_steps):
                # Left foot trajectory points
                left_idx = (current_idx + i) % num_steps
                left_pos = self.gait_trajectory[left_idx]
                left_point = Point()
                left_point.x = left_pos[0]
                left_point.y = 0.1  # Lateral offset for left foot
                left_point.z = self.hip_height - left_pos[1]
                left_marker.points.append(left_point)
                
                # Right foot trajectory points
                right_idx = (current_idx + i + num_steps // 2) % num_steps
                right_pos = self.gait_trajectory[right_idx]
                right_point = Point()
                right_point.x = right_pos[0]
                right_point.y = -0.1  # Lateral offset for right foot
                right_point.z = self.hip_height - right_pos[1]
                right_marker.points.append(right_point)
        
        # Add current position markers (spheres)
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
        current_left_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        
        current_left_marker.pose.position.x = self.left_foot[0]
        current_left_marker.pose.position.y = 0.1
        current_left_marker.pose.position.z = self.hip_height - self.left_foot[1]
        
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
        current_right_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan
        
        current_right_marker.pose.position.x = self.right_foot[0]
        current_right_marker.pose.position.y = -0.1
        current_right_marker.pose.position.z = self.hip_height - self.right_foot[1]
        
        # Add all markers to the array
        marker_array.markers.append(left_marker)
        marker_array.markers.append(right_marker)
        marker_array.markers.append(current_left_marker)
        marker_array.markers.append(current_right_marker)
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)


def main(args=None):
    # Initialize ROS and create node
    rclpy.init(args=args)
    
    # Initialize QApplication
    app = QApplication(sys.argv)
    
    # Create GUI
    gui = FootPositionGUI()
    gui.show()
    
    # Create the ROS node with a reference to the GUI
    node = GaitIKPublisher(gui)
    
    # Create executor for processing ROS callbacks in the background
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Timer to process ROS callbacks
    ros_timer = QTimer()
    def process_ros():
        executor.spin_once(timeout_sec=0)
    ros_timer.timeout.connect(process_ros)
    ros_timer.start(10)  # Process ROS callbacks every 10ms
    
    # Start Qt event loop
    exit_code = app.exec_()
    
    # Clean up ROS resources
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()