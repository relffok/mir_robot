import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time



class MirRestAPIClient(Node):

    def __init__(self):
        super().__init__('mir_restapi_client')
        self.create_api_clients()

        self.set_mir_time()

    def create_api_clients(self):
        self.restAPI_setTime = self.create_client(
            Trigger,
            'mir100_setTime')

    def set_mir_time(self):
        self.get_logger().info("calling time set service")
        req = Trigger.Request()
        future = self.restAPI_setTime.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60)
        if not future.done():
            self.get_logger().error("timeout")
        else:
            # service done
            self.get_logger().info("service executed")
            res = future.result()
            if res.success:
                self.get_logger().info(res.message)
            else:
                self.get_logger().error(res.message)


def main(args=None):
    rclpy.init(args=args)

    rest_api_client = MirRestAPIClient()

    rclpy.spin(rest_api_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()