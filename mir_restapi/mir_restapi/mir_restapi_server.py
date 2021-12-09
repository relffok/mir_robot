import time
import sys

import rclpy
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
from std_srvs.srv import Trigger


class MirRestAPIServer(Node):

    def __init__(self, hostname, auth):
        super().__init__('mir_restapi_server')
        self.get_logger().info("mir_restapi_server started")

        self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
            self.get_logger(), hostname, auth)
        self.get_logger().info("created MirRestAPI handle")

        self.create_api_services()
        
    def create_api_services(self):
        self.restAPI_setTime = self.create_service(
            Trigger,
            'mir100_setTime',
            self.api_setTime_callback)
        self.get_logger().info("Listening on 'mir100_setTime' for timeset call!")
    
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

    def api_setTime_callback(self, request, response):
        self.get_logger().info('Attempting to setTime through REST API...')
        self.connectRESTapi()
        
        # Request
        if self.api_handle.isConnected(printSuccess=False):
            self.api_handle.setDateTime() # produces an unavoidable connection timeout
            self.api_handle.close()
            response.success = True
            response.message = "Set Time succesfully"
        else:
            response.success = False
            response.message = "ERROR: Setting Time failed"


def main(args=None):
    rclpy.init(args=args)

    hostname = "192.168.12.20"
    auth = ""
    mir_restapi_server = MirRestAPIServer(hostname, auth)

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()