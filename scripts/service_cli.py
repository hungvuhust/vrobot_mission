#! /bin/bash/ python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from vrobot_route_follow.srv import MoveToPose


class ServiceCli(Node):
    def __init__(self):
        super().__init__("service_cli")

        self.get_logger().info("Service client node initialized")

        self.init_client()

    def init_client(self):
        self.ser_move_to_pose = self.create_client(
            MoveToPose, '/vrobot/move/go')

    def move_to_pose(self, map: str, id: int):
        while not self.ser_move_to_pose.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = MoveToPose.Request()
        response = MoveToPose.Response()

        request.map_name = map
        request.target_node_id = id

        future = self.ser_move_to_pose.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f"Service /vrobot/move/go result: {future.result()}")
            if (future.result().error_code != 0):
                raise Exception("ERROR")
        else:
            self.get_logger().info('exception while calling service: %r' %
                                   future.exception())
            raise Exception("ERROR")
