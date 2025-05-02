#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QTimer

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointSlider(QWidget):
    def __init__(self, label_text, joint_index, update_callback, parent=None):
        super().__init__(parent)
        self.joint_index = joint_index
        self.update_callback = update_callback

        self.label = QLabel(label_text)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(-157)  # -1.57 rad
        self.slider.setMaximum(157)   # 1.57 rad
        self.slider.setValue(0)
        self.slider.setTickInterval(1)

        self.value_label = QLabel("0.00")

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)
        self.setLayout(layout)

        self.slider.valueChanged.connect(self.update_label)
        self.slider.sliderReleased.connect(self.send_command_on_release)

    def update_label(self, value):
        self.value_label.setText(f"{value / 100:.2f}")

    def get_value(self):
        return self.slider.value() / 100.0

    def send_command_on_release(self):
        self.update_callback()


class JointPublisher(Node):
    def __init__(self):
        super().__init__('leg_slider_gui')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/leg_joint_trajectory_controller/joint_trajectory',
            10
        )

    def publish_command(self, positions):
        msg = JointTrajectory()
        msg.joint_names = [
            'left_hip1_joint',
            'left_hip2_joint',
            'left_knee_joint',
            'right_hip1_joint',
            'right_hip2_joint',
            'right_knee_joint',
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        msg.points.append(point)
        self.publisher.publish(msg)


class SliderApp(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle('Leg Joint Control')

        layout = QVBoxLayout()

        self.sliders = []
        for idx, name in enumerate([
            "left_hip1_joint",
            "left_hip2_joint",
            "left_knee_joint",
            "right_hip1_joint",
            "right_hip2_joint",
            "right_knee_joint"
        ]):
            slider = JointSlider(name, idx, self.send_command)
            self.sliders.append(slider)
            layout.addWidget(slider)

        self.setLayout(layout)

    def send_command(self):
        positions = [s.get_value() for s in self.sliders]
        self.node.publish_command(positions)


def main():
    rclpy.init()
    ros_node = JointPublisher()

    app = QApplication(sys.argv)
    gui = SliderApp(ros_node)
    gui.show()

    # QTimer to spin ROS in the Qt main loop
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
