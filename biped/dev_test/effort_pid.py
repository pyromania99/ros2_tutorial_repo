#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

class PIDController:
    def __init__(self, p, i, d):
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def compute(self, target, current):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 1e-5

        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time
        return output

class PIDEffortPublisher(Node):
    
    def __init__(self):
        super().__init__('pid_effort_publisher')

        self.joint_names = [
            'left_hip1_joint',
            'left_hip2_joint',
            'left_knee_joint',
            'right_hip1_joint',
            'right_hip2_joint',
            'right_knee_joint'
        ]

        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}

        self.pid_controllers = {
            name: PIDController(50.0, 0.0, 0) for name in self.joint_names
        }

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.target_sub = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def target_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn('Received target_positions length mismatch')
            return
        for i, name in enumerate(self.joint_names):
            self.target_positions[name] = msg.data[i]

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos

    def control_loop(self):
        efforts = []
        for name in self.joint_names:
            current = self.current_positions.get(name, 0.0)
            target = self.target_positions.get(name, 0.0)
            pid = self.pid_controllers[name]
            effort = pid.compute(target, current)
            efforts.append(effort)

        msg = Float64MultiArray()
        msg.data = efforts
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDEffortPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
