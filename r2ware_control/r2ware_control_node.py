import rclpy
from rclpy.node import Node

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    GearCommand,
    HazardLightsCommand,
    TurnIndicatorsCommand
)
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from geometry_msgs.msg import Twist

from Rosmaster_Lib import Rosmaster

import math


class YahboomCarDriver(Node):

    def __init__(self):
        super().__init__('r2ware_control_node')

        # Hardware driver
        self.car = Rosmaster()

        # State tracking
        self.brake = False
        self.left_turn = False
        self.right_turn = False
        self.reverse = False
        self.park = False
        self.emergency = False

        # For timeout detection
        self.last_cmd_time = self.get_clock().now()
        self.last_speed_cmd = 0.0

        # SUBSCRIBERS
        self.control_sub = self.create_subscription(
            Control, '/control/command/control_cmd',
            self.control_cmd_callback, 10)

        self.gear_sub = self.create_subscription(
            GearCommand, '/control/command/gear_cmd',
            self.gear_cmd_callback, 10)

        self.hazard_sub = self.create_subscription(
            HazardLightsCommand, '/control/command/hazard_lights_cmd',
            self.hazard_cmd_callback, 10)

        self.turn_sub = self.create_subscription(
            TurnIndicatorsCommand, '/control/command/turn_indicators_cmd',
            self.turn_cmd_callback, 10)

        self.emergency_cmd_sub = self.create_subscription(
            VehicleEmergencyStamped, '/control/command/emergency_cmd',
            self.autoware_emergency_cmd, 10)

        # PUBLISHERS
        self.vel_pub = self.create_publisher(Twist, '/vel_raw', 10)

        # TIMER FOR TIMEOUT CHECK
        self.timeout_timer = self.create_timer(0.05, self.check_timeout)

        self.get_logger().info("R2WARE control node started")


    # ======================================
    # Autoware CALLBACKS
    # ======================================

    def control_cmd_callback(self, msg):
        """Called when Autoware sends a new control command"""

        # Update timestamp to avoid timeout
        self.last_cmd_time = self.get_clock().now()
        self.last_speed_cmd = msg.longitudinal.velocity

        # Apply vehicle motion
        self.process_motion(msg.longitudinal.velocity,
                            msg.lateral.steering_tire_angle)

        # Update LEDs
        self.control_led_strip(
            brake=self.brake,
            left_turn=self.left_turn,
            right_turn=self.right_turn,
            reverse=self.reverse,
            emergency=self.emergency,
            park=self.park
        )


    def gear_cmd_callback(self, msg):
        if msg.command == GearCommand.REVERSE:
            self.reverse = True
            self.park = False
        elif msg.command == GearCommand.PARK:
            self.reverse = False
            self.park = True
        else:
            self.reverse = False
            self.park = False


    def hazard_cmd_callback(self, msg):
        if msg.command == HazardLightsCommand.ENABLE:
            self.emergency = True
        else:
            self.emergency = False


    def turn_cmd_callback(self, msg):
        if msg.command == TurnIndicatorsCommand.ENABLE_LEFT:
            self.left_turn = True
            self.right_turn = False
        elif msg.command == TurnIndicatorsCommand.ENABLE_RIGHT:
            self.left_turn = False
            self.right_turn = True
        else:
            self.left_turn = False
            self.right_turn = False


    def autoware_emergency_cmd(self, msg):
        """Autoware explicitly requests emergency stop"""
        self.emergency = msg.emergency
        if msg.emergency:
            self.car.set_car_motion(0.0, 0.0, 0.0)


    # ======================================
    # TIMEOUT & SAFETY HANDLING
    # ======================================

    def check_timeout(self):
        """If Autoware stops sending control_cmd, safely stop the robot without false emergency"""

        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_cmd_time.nanoseconds) / 1e9

        # IF Autoware stops sending commands:
        if dt > 0.5:

            # If robot is basically stopped → NORMAL situation (goal reached)
            if abs(self.last_speed_cmd) < 0.05:
                # No emergency. Just brake.
                self.brake = True
                self.emergency = False
                self.car.set_car_motion(0.0, 0.0, 0.0)
                return

            # If robot was moving, but control commands stopped → REAL EMERGENCY
            self.get_logger().warn("[SAFETY] Lost control commands while moving → EMERGENCY STOP")
            self.emergency = True
            self.car.set_car_motion(0.0, 0.0, 0.0)


    # ======================================
    # VEHICLE MOTION PROCESSING
    # ======================================

    def process_motion(self, speed, angle_rad):
        # self.get_logger().info(f"Speed={speed:.2f} m/s")

        # Steering scaling
        autoware_min = -0.7
        autoware_max = 0.7
        yamaha_min = -0.45
        yamaha_max = 0.45

        steering_angle = (
            (angle_rad - autoware_min)
            / (autoware_max - autoware_min)
            * (yamaha_max - yamaha_min)
            + yamaha_min
        )

        # Clamp Autoware steering to robot limits
        max_robot_angle = 0.45
        steering_angle = max(min(angle_rad, max_robot_angle), -max_robot_angle)

        # Brake detection
        self.brake = True if speed == 0 else False

        # Send to robot
        self.car.set_car_motion(speed, steering_angle, 0.0)

        # Publish velocity feedback
        vel_msg = Twist()
        vel_msg.linear.x = float(speed)
        vel_msg.angular.z = 0.0
        self.vel_pub.publish(vel_msg)


    # ======================================
    # LED STRIP HANDLING
    # ======================================

    def control_led_strip(self, brake=False, left_turn=False, right_turn=False,
                          reverse=False, emergency=False, park=False):

        colors = [(0, 0, 0)] * 14

        if left_turn:
            for led in [0, 1]:
                colors[led] = (255, 80, 0)

        if brake:
            for led in [3, 4, 5, 6, 7, 8, 9, 10]:
                colors[led] = (255, 0, 0)

        if reverse:
            for led in [2, 11]:
                colors[led] = (255, 255, 255)

        if right_turn:
            for led in [12, 13]:
                colors[led] = (255, 80, 0)

        if emergency:
            for i in range(14):
                colors[i] = (255, 0, 0)

        if park:
            for i in range(14):
                colors[i] = (0, 255, 0)

        for i, (r, g, b) in enumerate(colors):
            self.car.set_colorful_lamps(i, r, g, b)


def main():
    rclpy.init()
    node = YahboomCarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.car.set_car_motion(0.0, 0.0, 0.0)
        node.car.set_colorful_lamps(0xff, 0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
