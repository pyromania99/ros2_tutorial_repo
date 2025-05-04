#!/usr/bin/env python3

import os
import subprocess
import threading

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
import keyboard  # pip install keyboard


class ControllerSwitcher(QWidget):
    def __init__(self):
        super().__init__()
        self.current_controller = "effort_controller"
        self.setWindowTitle("Controller Switcher")
        self.label = QLabel(f"Active Controller: {self.current_controller}")
        self.label.setAlignment(Qt.AlignCenter)

        self.toggle_button = QPushButton("Toggle Controller (Hotkey: F1)")
        self.toggle_button.clicked.connect(self.toggle_controller)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.toggle_button)
        self.setLayout(layout)

        # Listen for hotkey in background
        threading.Thread(target=self.listen_hotkey, daemon=True).start()

    def toggle_controller(self):
        if self.current_controller == "leg_joint_trajectory_controller":
            self.switch_controller("effort_controller")
        else:
            self.switch_controller("leg_joint_trajectory_controller")

    def switch_controller(self, target_controller):
        controllers = ["leg_joint_trajectory_controller", "effort_controller"]
        to_stop = [c for c in controllers if c != target_controller]

        for c in to_stop:
            subprocess.run(["ros2", "control", "set_controller_state", c, "inactive"])
        subprocess.run(["ros2", "control", "set_controller_state", target_controller, "active"])

        self.current_controller = target_controller
        self.label.setText(f"Active Controller: {self.current_controller}")

    def listen_hotkey(self):
        keyboard.add_hotkey('F1', self.toggle_controller)
        while True:
            if not self.isVisible():
                keyboard.unhook_all_hotkeys()
                break
            keyboard.wait('F1')  # Wait only for F1 then loop again


def main():
    app = QApplication([])
    switcher = ControllerSwitcher()
    switcher.resize(300, 100)

    def cleanup():
        keyboard.unhook_all_hotkeys()

    app.aboutToQuit.connect(cleanup)

    switcher.show()
    app.exec_()

    

if __name__ == '__main__':
    main()
