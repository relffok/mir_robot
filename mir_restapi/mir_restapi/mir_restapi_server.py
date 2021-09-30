import time
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
import mir_restapi.action


class MirRestAPIActionServer(Node):

    def __init__(self, hostname, auth):
        super().__init__('mir_restapi')

        self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
            self.get_logger(), hostname, auth)

        # ===
        # Action server definitions
        # ===
        self.mir100_setTime_action_server = ActionServer(
            self,
            mir_restapi.action.mir100_setTime,
            'mir100_setTime',
            self.mir100_setTime)
    
    def connectRESTapi(self):
        self.get_logger().info('REST API: Waiting for connection')
        i = 1
        while not self.api_handle.isConnected():
            if not rclpy.ok():
                sys.exit(0)
            if i > 5:
                self.get_logger().error('REST API: Could not connect, giving up')
                break
            i += 1
            time.sleep(1)


    # ===
    # Action server callbacks
    # ===

    def mir100_setTime(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # Feedback
        feedback_msg = mir_restapi.action.mir100_setTime.Feedback()
        feedback_msg.settingTime = "true"
        self.get_logger().info('Feedback: {}'.format(feedback_msg.settingTime))
        goal_handle.publish_feedback(feedback_msg)

        self.connectRESTapi()
        
        # options:
        # goal_handle.request.options

        # Request
        if self.api_handle.isConnected(printSuccess=False):
            self.api_handle.setDateTime() # produces an unavoidable connection timeout
            self.api_handle.close()

        goal_handle.succeed()

        # Result
        result = mir_restapi.action.mir100_setTime.Result()
        result.setTime = "true"
        return result


def main(args=None):
    rclpy.init(args=args)

    hostname = "192.168.12.20"
    auth = ""
    mir_restapi_server = MirRestAPIActionServer(hostname, auth)

    rclpy.spin(mir_restapi_server)


if __name__ == '__main__':
    main()