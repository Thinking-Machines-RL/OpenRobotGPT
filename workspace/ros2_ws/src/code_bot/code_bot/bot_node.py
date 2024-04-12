import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution
from .bots import DecisionBot, CorrectionBot, EvaluationBot, ChatGPT
import pkg_resources
import re
import os

class BotNode(Node):
    def __init__(self):
        super().__init__('bot_node')
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
    node = BotNode()

    config_path = "/root/workspace/ros2_ws/install/code_bot/share/code_bot/config/config_bot.json"
    secret_path = os.environ.get("API_KEY_PATH")
    decision_bot = ChatGPT(config_path, secret_path)
    evaluation_bot = ChatGPT(config_path, secret_path)
    correction_bot = ChatGPT(config_path, secret_path)

    while True:
        task = input("Enter the task: ")

        decision_instructions = "You will be asked to produce code that solves a task. Only write code. Every kind of explanation should be included as python comments. Your entire response should be directly executable in a python shell."
        decision_bot.set_context([{"role":"user", "content":decision_instructions}])
        code = decision_bot.chat(task)

        print("ChatGPT code (raw): ", code)
        x = input("Press a key to proceed or type 'QUIT' to quit: ")
        if x.strip().upper() == 'QUIT':
            break

        # Clean the code from the overhead
        pattern = r"```python(.*?)```"
        code = re.findall(pattern, code, re.DOTALL)[0]

        print("ChatGPT code (cleaned): ", code)
        x = input("Press a key to proceed or type 'QUIT' to quit: ")
        if x.strip().upper() == 'QUIT':
            break

        evaluation_instructions = "You will be prompted with a task. The task has already been solved. Your task is to check if the result is correct, without modifying it. In order to do that we need you to provide a python function (ONLY this function) called evaluation_func() that takes no arguments and checks if the results of the prevoius program (that you do not need to compute and are already available as global variables) are correct. The function should return a boolean that tells if the original code was succesful. Only write code. Every kind of explanation should be included as python comments. Your entire response should be directly executable in a python shell."
        evaluation_bot.set_context([{"role":"user", "content":evaluation_instructions}])
        evaluation_code = evaluation_bot.chat(task)

        print("ChatGPT evaluation code (raw): ", evaluation_code)
        x = input("Press a key to proceed or type 'QUIT' to quit: ")
        if x.strip().upper() == 'QUIT':
            break

        # Clean the code from the overhead
        pattern = r"```python(.*?)```"
        evaluation_code_list = re.findall(pattern, evaluation_code, re.DOTALL)
        if evaluation_code_list:
            evaluation_code = evaluation_code_list[0]

        print("ChatGPT evaluation code (cleaned): ", evaluation_code)
        x = input("Press a key to proceed or type 'QUIT' to quit: ")
        if x.strip().upper() == 'QUIT':
            break

        code_is_working = node.call_service(code, evaluation_code)

        if code_is_working:
            print("WORKING!")
        else:
            print("NOT working!")

        x = input("Press a key to proceed or type 'QUIT' to quit: ")
        if x.strip().upper() == 'QUIT':
            break


    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

