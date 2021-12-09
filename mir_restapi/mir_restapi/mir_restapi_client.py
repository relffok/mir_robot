import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger



class MirRestAPIClient(Node):

    def __init__(self):
        super().__init__('mir_restapi_client')
        self.create_api_clients()

    def create_api_clients(self):
        self.restAPI_setTime = self.create_client(
            Trigger,
            'mir100_setTime')


def main(args=None):
    rclpy.init(args=args)

    rest_api_client = MirRestAPIClient()

    rclpy.spin(rest_api_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()