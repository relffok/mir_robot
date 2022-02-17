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
        
        if self.api_handle == None:
            self.get_logger().warn("""
            Hostname and API token are not set! Run as follows: 
            
            ros2 run mir_restapi mir_restapi_server --ros-args -p mir_hostname:='MIR_IP_ADDR' -p mir_restapi_auth:='YOUR_API_KEY'
            """)
    
    def setup_api_handle(self):
        if self.hostname != "" and self.auth != "":
            self.api_handle = mir_restapi.mir_restapi_lib.MirRestAPI(
                self.get_logger(), self.hostname, self.auth)
            self.get_logger().info("created MirRestAPI handle")
            self.create_services()
            self.get_logger().info("created services")
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == "mir_restapi_auth":
                self.get_logger().info("Received auth token")
                self.auth = param.value
            if param.name == "mir_hostname":
                self.get_logger().info("Set mir hostname")
                self.hostname = param.value
        self.setup_api_handle()
        return SetParametersResult(successful=True)
    
    def create_services(self):
        self.restAPI_setTime = self.create_service(
            Trigger,
            'mir_100_sync_time',
            self.sync_time_callback)
        self.get_logger().info("Listening on 'mir_100_sync_time'")

        self.restAPI_get_status = self.create_service(
            Trigger,
            'mir_100_get_status',
            self.get_status_callback)
        self.get_logger().info("Listening on 'mir_100_get_status'")

        self.restAPI_get_sounds = self.create_service(
            Trigger,
            'mir_100_get_sounds',
            self.get_sounds_callback)
        self.get_logger().info("Listening on 'mir_100_get_sounds'")

        self.restAPI_is_emergency_halt = self.create_service(
            Trigger,
            'mir_100_is_emergency_halt',
            self.is_emergency_halt_callback)
        self.get_logger().info("Listening on 'mir_100_is_emergency_halt'")

        self.restAPI_get_missions = self.create_service(
            Trigger,
            'mir_100_get_missions',
            self.get_missions_callback)
        self.get_logger().info("Listening on 'mir_100_get_missions'")

        self.restAPI_honk = self.create_service(
            Trigger,
            'mir_100_honk',
            self.honk)
        self.get_logger().info("Listening on 'mir_100_honk'")
    
    def test_api_connection(self):
        if self.api_handle == None:
            return -1
        
        self.get_logger().info('REST API: Waiting for connection')
        i = 1
        while not self.api_handle.is_connected():
            if not rclpy.ok():
                sys.exit(0)
            if i > 5:
                self.get_logger().error('REST API: Could not connect, giving up')
                return 0
            i += 1
            time.sleep(1)
        return 1
    
    def reponse_api_handle_not_exists(self, response):
        response.success = False
        response.message = 'API token and/or hostname not set yet'
        self.get_logger().error(response.message)
        return response
    
    def call_restapi_function(self, service_fct, request, response, args=None):
        if self.test_api_connection() == -1:
            response = self.reponse_api_handle_not_exists(response)
            return response
        if self.api_handle.is_connected(print=False):
            if args==None:
                response.message = str(service_fct())
            else:
                response.message = str(service_fct(args))
            if "Error" in response.message:
                response.success = False
            else:
                response.success = True
            return response
        else:
            response.success = False
            response.message = "ERROR: Couldn't connect to REST API"
        self.get_logger().error(response.message)
        return response

    def sync_time_callback(self, request, response):
        self.get_logger().info('Syncing host time with REST API...')
        response = self.call_restapi_function(self.api_handle.sync_time, request, response)
        return response

    def get_status_callback(self, request, response):
        self.get_logger().info('Getting status from REST API...')
        response = self.call_restapi_function(self.api_handle.get_status, request, response)
        return response
    
    def get_sounds_callback(self, request, response):
        self.get_logger().info('Getting sounds from REST API...')
        response = self.call_restapi_function(self.api_handle.get_sounds, request, response)
        return response
    
    def is_emergency_halt_callback(self, request, response):
        self.get_logger().info('Checking REST API for emergency halt...')
        response = self.call_restapi_function(self.api_handle.get_state_id, request, response)
        
        if response.success:
            state_id = int(response.message)
            self.get_logger().info("Returned state_id as %i" % state_id)
            STATE_ID_EMERGENCY = 10
            if state_id == STATE_ID_EMERGENCY:
                response.message = str(True)
            else:
                response.message = str(False)
        return response
    
    def get_missions_callback(self, request, response):
        self.get_logger().info('Getting missions from REST API...')
        response = self.call_restapi_function(self.api_handle.get_missions, request, response)
        return response
    
    def honk(self, request, response):
        self.get_logger().info('Honking horn over REST API...')

        mission_name = "honk"

        response_miss_queue = self.call_restapi_function(self.api_handle.add_mission_to_queue, request, response, args=mission_name)
        if response_miss_queue.message == str(False):
            response.message = "Honking failed due to mission queue error"
            self.get_logger().error(response.message)
            response.success = False
            return response
        
        emerg_response = self.is_emergency_halt_callback(request, response)
        if emerg_response.message == str(True):
            response.message = "Can't honk, emergency halt"
            self.get_logger().error(response.message)
            response.success = False
        else:
            response.message = "Honking"
            self.get_logger().info(response.message)
            STATE_ID_RUN_MISSION = 3
            STATE_ID_PAUSE = 4
            
            self.api_handle.set_state_id(STATE_ID_RUN_MISSION)
            
            # while self.api_handle.is_mission_queue_empty():
            time.sleep(5)
            
            self.api_handle.set_state_id(STATE_ID_PAUSE)
            response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    mir_restapi_server = MirRestAPIServer()

    rclpy.spin(mir_restapi_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()