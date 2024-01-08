import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        self.setpoint_velocity = 5.0
        self.current_velocity = 0.0
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.01

        self.control_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Serial port configuration for STM32 communication
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Update with your STM32's serial port

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x
        self.calculate_pid()
        self.publish_control_command()

    def calculate_pid(self):
        self.error = self.setpoint_velocity - self.current_velocity
        self.integral += self.error
        derivative = self.error - self.prev_error

        pid_output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = self.error

        # Limit the output to a reasonable range
        pid_output = max(min(pid_output, 1.0), -1.0)

        self.control_command = Twist()
        self.control_command.linear.x = pid_output

    def publish_control_command(self):
        # Send PID output to STM32 for motor control
        control_command = f'M{int(self.control_command.linear.x * 1000)}\n'  # Adjust as needed
        self.ser.write(control_command.encode())

        # Publish control command for visualization or logging
        self.control_pub.publish(self.control_command)

def main(args=None):
    rclpy.init(args=args)

    controller = VelocityController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
