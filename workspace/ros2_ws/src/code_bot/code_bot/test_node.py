import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution
from .bots import DecisionBot, CorrectionBot, EvaluationBot, ChatGPT
import pkg_resources
import json
import re
import os
import json

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        print('Hello from bot_node!')

        # TODO: tune
        self.SERVICE_TIMEOUT = 60

        self.client = self.create_client(CodeExecution, 'test_code')
        while not self.client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Service not available, waiting again...')
        self.req = CodeExecution.Request()

    def call_service(self, code: str, evaluation_code: str):
        self.req.code = code
        self.req.evaluation_code = evaluation_code
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().completion_flag, future.result().code_except, future.result().eval_except

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    code = """def execution_func(self):
        _, _ , _ , done =self.pick([0.6, 0.1, 0.05, 1, 0, 0, 0])
        self.place([0.7, 0.1, 0.05, 1, 0, 0, 0])
    """
    evaluation_code = """def valuation_func(self):
        return True
    """
    code_is_working = node.call_service(code, evaluation_code)

    if code_is_working:
        print("WORKING!")
    else:
        print("NOT working!")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

