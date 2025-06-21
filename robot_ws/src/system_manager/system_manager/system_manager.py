import structlog
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy
)

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sys_msgs.msg import RobotInfo

logger = structlog.get_logger()

class SystemManager(Node):

    def __init__(self):
        super().__init__(node_name="system_manager")
        self._register_publisher()
        self._register_timer()
    
    def _register_publisher(self):

        self._device_info_pub = self.create_publisher(msg_type=RobotInfo,
                                                      topic="/robot_device_info",
                                                      qos_profile=QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                                                             reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                                             durability=QoSDurabilityPolicy.VOLATILE,
                                                                             depth=5))

    def _register_timer(self):

        self._pub_timer = self.create_timer(timer_period_sec=1.0,
                                            callback=self._publish_robot_info,
                                            callback_group=MutuallyExclusiveCallbackGroup())

    def _publish_robot_info(self):

        logger.info("publish robot info")
        self._device_info_pub.publish(RobotInfo(robot_id="smr01",
                                                robot_name="smr01"))
    


def main(args=None):
    rclpy.init(args=args)
    system_manager_node = SystemManager()
    rclpy.spin(system_manager_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    system_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()