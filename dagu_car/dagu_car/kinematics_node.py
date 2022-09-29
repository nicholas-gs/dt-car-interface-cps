#!/usr/bin/env python3

import json
import yaml
import rclpy
import os.path

from rclpy.node import Node

import dt_calibration_utils.calibration_file as calibration_utils

from dt_interfaces_cps.msg import (
    Twist2DStamped,
    WheelsCmdStamped
)


class KinematicsNode(Node):
    REL_CALIBRATION_DIR = "kinematics"

    # List of all configuration names
    CONFIGURATION_NAMES = [
        "gain",
        "trim",
        "baseline",
        "radius",
        "k",
        "limit",
        "v_max",
        "omega_max"
    ]

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.load_launch_parameters()
        self.read_params_from_calibration_file(self._veh_name)
        self.set_config_as_ros_params()

        self.get_logger().info(f"Initialized with"
            f" {json.dumps(self.get_current_configuration(),sort_keys=True, indent=4)}")

        self.pub_wheels_cmd = self.create_publisher(
            WheelsCmdStamped,
            "~/wheels_cmd",
            1)
        self.pub_velocity = self.create_publisher(
            Twist2DStamped,
            "~/velocity",
            1)
        self.sub_car_cmd = self.create_subscription(
            Twist2DStamped,
            "~/car_cmd",
            self.car_cmd_callback,
            1)

    def load_launch_parameters(self):
        self.declare_parameter("veh")
        self.declare_parameter("default_config")

        self._veh_name = self.get_parameter("veh").get_parameter_value()\
            .string_value
        self._default_config_fp = self.get_parameter("default_config")\
            .get_parameter_value().string_value

    def set_config_as_ros_params(self):
        """Take the configuration parameters that was loaded from YAML file
        and declare them as ROS2 parameters for easier introspection.
        """
        self.declare_parameters("", [(c,) for c in self.CONFIGURATION_NAMES])
        all_new_parameters = [rclpy.parameter.Parameter(
                c,
                rclpy.Parameter.Type.DOUBLE,
                getattr(self, f"_{c}")
            ) for c in self.CONFIGURATION_NAMES]
        self.set_parameters(all_new_parameters)

    def get_current_configuration(self):
        """Get the current configuration values being used."""
        return {config : getattr(self, f"_{config}") 
            for config in self.CONFIGURATION_NAMES}

    def read_params_from_calibration_file(self, veh: str):
        """Read in calibration file.
        First try to read in calibration file from the specific to the robot.
        If unable to find or read, then use the default calibration file.
        """
        calibration_data = None
        calibration_data, full_fp, _ = calibration_utils.read_calibration(
            os.path.join(self.REL_CALIBRATION_DIR, f"{veh}.yaml"))

        if calibration_data is None:
            self.get_logger().info(f"Unable to find calibration data at {full_fp}")
            # Load default calibration file
            with open(self._default_config_fp, 'r') as f:
                try:
                    calibration_data = yaml.safe_load(f)
                    self.get_logger().info(
                        f"Read defaul calibration file {self._default_config_fp}")
                except yaml.YAMLError as _:
                    msg = f"Unable to read in default calibration file {self._default_config_fp}"
                    self.get_logger().fatal(msg)
                    raise RuntimeError(msg)
        else:
            self.get_logger().info(f"Using calibration file {full_fp}")

        for config_name in self.CONFIGURATION_NAMES:
            setattr(self, f"_{config_name}", calibration_data[config_name])

    def car_cmd_callback(self, msg_car_cmd: Twist2DStamped):
        """A callback that reposponds to received `car_cmd` messages by
        calculating the corresponding wheel commands, taking into account the
        robot geometry, gain and trim factors, and the set limits.
        These wheel commands are then published for the motors to use.
        The resulting linear and angular velocities are also calculated
        and published.
        Args:
            msg_car_cmd (:obj:`Twist2DStamped`): desired car command
        """
        # INVERSE KINEMATICS PART

        # trim the desired commands such that they are within the limits:
        msg_car_cmd.v = self.trim(
            msg_car_cmd.v, low=-self._v_max,
            high=self._v_max)
        msg_car_cmd.omega = self.trim(
            msg_car_cmd.omega, low=-self._omega_max,
            high=self._omega_max
        )

        # assuming same motor constants k for both motors
        k_r = k_l = self._k

        # adjusting k by gain and trim
        k_r_inv = (self._gain + self._trim) / k_r
        k_l_inv = (self._gain - self._trim) / k_l

        omega_r = ((msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self._baseline) 
            / self._radius)
        omega_l = ((msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self._baseline) 
            / self._radius)

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self._limit, self._limit)
        u_l_limited = self.trim(u_l, -self._limit, self._limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)  

        # FORWARD KINEMATICS PART

        # Conversion from motor duty to motor rotation rate
        omega_r = msg_wheels_cmd.vel_right / k_r_inv
        omega_l = msg_wheels_cmd.vel_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (self._radius * omega_r + self._radius * omega_l) / 2.0
        omega = ((self._radius * omega_r - self._radius * omega_l) 
            / self._baseline)

        # Put the v and omega into a velocity message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = omega
        self.pub_velocity.publish(msg_velocity)

    @staticmethod
    def trim(value, low, high):
        """Trims a value to be between some bounds.
        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound
        Returns:
            the trimmed value
        """
        return max(min(value, high), low)        


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode("kinematics_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
