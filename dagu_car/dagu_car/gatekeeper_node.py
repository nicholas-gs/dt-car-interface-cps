#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from dt_interfaces_cps.msg import BoolStamped, Twist2DStamped


class GatekeeperNode(Node):
    """Gatekeeps the `car_cmd` before being sent to the `kinematics_node`.
    Either stops or passthrough the `car_cmd` messages.
    """
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.activated = False

        # Subscribers
        self.sub_car_cmds = self.create_subscription(
            Twist2DStamped,
            "~/in_car_cmd",
            self.cb_car_cmd,
            1)
        self.sub_start = self.create_subscription(
            BoolStamped,
            "~/activate",
            self.cb_start,
            1)

        # Publisher
        self.pub_car_cmd = self.create_publisher(
            Twist2DStamped,
            "~/out_car_cmd",
            1)

        self.get_logger().info(f"Activated with passthrough: {self.activated}")

    def cb_car_cmd(self, wheels_cmd: Twist2DStamped):
        """Callback for car commands."""
        if self.activated:
            self.pub_car_cmd.publish(wheels_cmd)

    def cb_start(self, msg: BoolStamped):
        self.activated = msg.data
        self.get_logger().info(f"Gatekeeper activated: {self.activated}")


def main(args=None):
    rclpy.init(args=args)
    node = GatekeeperNode("gatekeeper_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
