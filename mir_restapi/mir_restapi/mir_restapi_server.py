import time
import sys

import rclpy
from rclpy.node import Node

import mir_restapi.mir_restapi_lib
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


class MirRestAPIServer(Node):

    def __init__(self):
        super().__init__('mir_restapi_server')
        self.get_logger().info("started")

        # parameters: hostname, api_token
        self.declare_parameter('mir_hostname', "")
        self.hostname = self.get_parameter('mir_hostname').get_parameter_value().string_value
        self.declare_parameter('mir_restapi_auth', "")
        self.auth = self.get_parameter('mir_restapi_auth').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.api_handle = None
        self.setup_api_handle()

        self.create_services()
    
    def setup_api_handle(self):
        if self.hostname != "" and self.auth != "" and self.api_handle == None:
            self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
                self.get_logger(), self.hostname, self.auth)
            self.get_logger().info("created MirRestAPI handle")
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == "mir_restapi_auth":
                self.get_logger().info("Received auth token")
                self.auth = param.value
            if param.name == "mir_restapi_auth":
                self.get_logger().info("Set mir hostname")
                self.hostname = param.value
        self.setup_api_handle()
        return SetParametersResult(successful=True)
        
    def create_services(self):
        self.restAPI_setTime = self.create_service(
            Trigger,
            'mir100_setTime',
            self.api_setTime_callback)
        self.get_logger().info("Listening on 'mir100_setTime' for timeset call!")
    
    def connectRESTapi(self):
        if self.api_handle == None:
            return -1
        
        self.get_logger().info('REST API: Waiting for connection')
        i = 1
        while not self.api_handle.isConnected():
            if not rclpy.ok():
                sys.exit(0)
            if i > 5:
                self.get_logger().error('REST API: Could not connect, giving up')
                return 0
            i += 1
            time.sleep(1)
        return 1
    
    def reponse_api_handle_non_existing(self, response):
        response.success = False
        response.message = 'API token and/or hostname not set yet'
        self.get_logger().error(response.message)
        return response

    def api_setTime_callback(self, request, response):
        self.get_logger().info('Attempting to setTime through REST API...')
        
        if self.connectRESTapi() == -1:
            response = self.reponse_api_handle_non_existing(response)
            return response
        
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
            
            if "Error" in response.message:
                response.success = False
            else:
                response.success = True
        else:
            response.success = False
            response.message = "ERROR: Couldn't connect to REST API"
        self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    mir_restapi_server = MirRestAPIServer()

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()