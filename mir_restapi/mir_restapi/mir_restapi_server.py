import time
import sys

import rclpy
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


class MirRestAPIServer(Node):

    def __init__(self, hostname):
        super().__init__('mir_restapi_server')
        self.get_logger().info("mir_restapi_server started")
        
        self.hostname = hostname
        self.create_api_services()

        self.declare_parameter('auth', "")
        self.auth = self.get_parameter('auth').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == "auth":
                self.get_logger().info("Received auth token")
                self.auth = param.value
                self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
                    self.get_logger(), self.hostname, self.auth)
                self.get_logger().info("created MirRestAPI handle")
        return SetParametersResult(successful=True)
        
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
        if self.api_handle.isConnected(print=False):
            # produces an unavoidable connection timeout
            response.message = self.api_handle.setDateTime() 
            # this is needed, so that the connection timeout can be ignored
            self.get_logger().info('REST API: Waiting to close connection: 10s')
            time.sleep(5) 
            self.get_logger().info('REST API: Waiting to close connection: 5s')
            time.sleep(5)
            self.api_handle.close()
            
            response.success = True
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = "ERROR: Setting Time failed"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    hostname = "192.168.12.20"
    mir_restapi_server = MirRestAPIServer(hostname)

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()