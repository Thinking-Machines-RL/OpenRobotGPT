import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution


class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        self.srv = self.create_service(CodeExecution, 'test_code', self.test_callback)

    def test_callback(self, request, response):
        code = request.code
        evaluation_code = request.evaluation_code

        # Evaluate code
        exec(code, globals())

        # Define evaluation function
        scope = {}
        exec(evaluation_code, globals(), scope)

        completion_flag = scope['evaluation_func']()

        response.completion_flag = completion_flag
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

