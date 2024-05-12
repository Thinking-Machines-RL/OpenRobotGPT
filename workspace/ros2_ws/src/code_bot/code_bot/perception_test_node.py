import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution, EvaluationCode
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from robotgpt_interfaces.msg import ResultEvaluation, CodeExecutionM
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

        service_group = ReentrantCallbackGroup()
        self.exec_pub = self.create_publisher(CodeExecutionM, 'chat_gpt_bot/test_code', 10, callback_group = service_group)
        # self.client = self.create_client(CodeExecution, 'chat_gpt_bot/test_code')
        # while not self.client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
        #     self.get_logger().info('Service not available, waiting again...')
        # self.req = CodeExecution.Request()

        #service to generate evaluation code
        self.evaluation_service = self.create_service(EvaluationCode, 'chat_gpt_bot/evaluation_code', self.service_callback, callback_group = service_group)

        #topic with the results of the evaluation
        self.result_sub = self.create_subscription(ResultEvaluation, 'chat_gpt_bot/evaluation_results', self.result_callback, 10 )
    
    def result_callback(self, msg):
        print("message received")
        if msg.completion_flag == True:
            print("task completed")
        else:
            print("error evaluation", msg.eval_except)

    def call_service(self, code: str):
        # self.req.code = code
        # future = self.client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result().code_except
        msg = CodeExecutionM()
        msg.code = code
        self.exec_pub.publish(msg)
    
    def service_callback(self, request, response):

        evaluation_code = """def valuation_func(self):
            return True
        """
        response.evaluation_code = evaluation_code
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    code = """def execution_func(self):
        objStates = self.getInitialObjectStates()
        blue_cube_pos = objStates["blue_cube"]
        self.pick(blue_cube_pos + [1, 0, 0, 0])
        self.place([0.7, 0.1, 0.05, 1, 0, 0, 0], True)
        return True
    """
    node.call_service(code)

    # print("recived response ", code_is_working)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
