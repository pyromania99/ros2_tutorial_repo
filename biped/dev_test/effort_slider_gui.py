#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading


class EffortPublisherNode(Node):
    def __init__(self):
        super().__init__('effort_slider_gui')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.efforts = [0.0] * 6  # Initialize effort for 6 joints

    def publish_efforts(self, efforts):
        msg = Float64MultiArray()
        msg.data = efforts
        self.publisher_.publish(msg)


class EffortSliderGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle('Effort Controller Sliders')
        self.ros_node = ros_node
        self.num_joints = 6
        self.joint_names = [
            'left_hip1_joint', 'left_hip2_joint', 'left_knee_joint',
            'right_hip1_joint', 'right_hip2_joint', 'right_knee_joint'
        ]
        self.sliders = []
        self.init_ui()

        # Timer to publish efforts periodically
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_efforts)
        self.timer.start(100)  # Every 100 ms

    def init_ui(self):
        layout = QVBoxLayout()
        for i in range(self.num_joints):
            joint_layout = QHBoxLayout()
            label = QLabel(self.joint_names[i])
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-100)
            slider.setMaximum(100)
            slider.setValue(0)
            slider.setTickInterval(10)
            slider.setTickPosition(QSlider.TicksBelow)
            self.sliders.append(slider)

            joint_layout.addWidget(label)
            joint_layout.addWidget(slider)
            layout.addLayout(joint_layout)

        self.setLayout(layout)
        self.resize(400, 300)

    def send_efforts(self):
        efforts = [slider.value() / 10.0 for slider in self.sliders]  # Scale to -10 to 10
        self.ros_node.publish_efforts(efforts)


def run_gui():
    rclpy.init()
    ros_node = EffortPublisherNode()

    def ros_spin():
        rclpy.spin(ros_node)

    threading.Thread(target=ros_spin, daemon=True).start()

    app = QApplication(sys.argv)
    gui = EffortSliderGUI(ros_node)
    gui.show()
    app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run_gui()
