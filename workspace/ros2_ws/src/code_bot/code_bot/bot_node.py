import rclpy
from rclpy.node import Node
from messages.srv import CodeExecution
from bots import DecisionBot, CorrectionBot, EvaluationBot, ChatGPT

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
        future = self.client.call_async(self.req)
        return future.result().completion_flag
    


def main(args=None):
    rclpy.init(args=args)
    node = BotNode()

    decision_bot = ChatGPT("config/config_bot.json", "secrets/api_key.json")
    evaluation_bot = ChatGPT("config/config_bot.json", "secrets/api_key.json")
    correction_bot = ChatGPT("config/config_bot.json", "secrets/api_key.json")

    # Set context for each bot
    decision_context = "You are a robotics engineer, writing code for a franka robot. Examples: ..."
    decision_bot.set_context([{"role":"user", "content":decision_context}])

    evaluation_context = "You need to write a function that check if the code given as input is correct. Examples: ..."
    evaluation_bot.set_context([{"role":"user", "content":evaluation_context}])

    correction_context = "You need to explain why the provided code doesn't work. Examples: ..."
    correction_bot.set_context([{"role":"user", "content":correction_context}])

    # Prompt task
    task = input("What do you want OpenRobotGPT to do?")

    evaluation_code = evaluation_bot.chat("Task: {task}")

    old_code = ""
    report = ""
    code_is_working = False
    while not code_is_working:
        code = decision_bot.chat("Task: {task} | Old code: {old_code} | Report: {report}")
        code_is_working = node.call_service(code, evaluation_code)
        if not code_is_working:
            old_code = code
            report = correction_bot.chat("Task: {task} | Code: {code}")

    # Now we have working code in "code" variable



    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

