import time
import structlog

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .robot import Robot

from nav_interfaces.msg import RobotLocation
from nav_interfaces.srv import MoveToGoal

class Simulator(Node):

    def __init__(self):
        super().__init__(node_name="system_manager")

        # register log handler
        self.logger = structlog.get_logger()

        # register robot instance
        self._robot = Robot(logger=self.logger)

        # register publisher
        self.register_publishers()

        # register timer
        self.register_timer()

        # register services
        self.register_services()

    def register_publishers(self):

        self._location_pub = self.create_publisher(
            msg_type=RobotLocation,
            topic="robot/location",
            qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                   reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                   durability=QoSDurabilityPolicy.VOLATILE,
                                   depth=5)
        )
    
    def register_services(self):
        self._move_to_goal_srv = self.create_service(
            srv_type=MoveToGoal,
            srv_name="robot/move_to_goal",
            callback_group=MutuallyExclusiveCallbackGroup(),
            callback=self._move_to_goal_callback
        )

    def register_timer(self):
        self._location_timer = self.create_timer(timer_period_sec=0.5,
                                                 callback_group=MutuallyExclusiveCallbackGroup(),
                                                 callback=self._publish_location)
        
    def _publish_location(self):

        # get robot location
        location = self._robot.get_location()

        # create message
        msg = RobotLocation(timestamp=time.time(),
                            x=location.x, 
                            y=location.y, 
                            theta=location.theta)

        # publish message
        self._location_pub.publish(msg)

        # log message
        self.logger.info("[Simulator] Published robot location", x=location.x, y=location.y, theta=location.theta)

    def _move_to_goal_callback(self, request: MoveToGoal.Request, response: MoveToGoal.Response) -> MoveToGoal.Response:

        self.logger.info(f"[Simulator][move_to_goal_callback] Received move to goal request: [{request.goal_x}, {request.goal_y}]")

        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    simulator_node = Simulator()
    rclpy.spin(simulator_node)

if __name__ == "__main__":
    main()