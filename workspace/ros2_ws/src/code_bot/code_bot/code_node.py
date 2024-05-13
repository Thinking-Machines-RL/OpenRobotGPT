import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution, EvaluationCode, ObjectStatesR
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from robotgpt_interfaces.msg import ResultEvaluation, CodeExecutionM
from .bots import DecisionBot, CorrectionBot, EvaluationBot, ChatGPT
import pkg_resources
import json
import re
import os
import json


class CodeNode(Node):
    def __init__(self):
        super().__init__('code_node')
        print('Hello from code_node!\nUse this node to generate code that solves a manipulation task')


        # Constants
        # TODO: tune
        self.SERVICE_TIMEOUT = 60
        self.MAX_ATTEMPTS = 10

        # Initialize attempt counter
        self.attemps = 0

        # Create callback group
        service_group = ReentrantCallbackGroup()

        # Load config files and secrets
        config_path = "/root/workspace/ros2_ws/install/code_bot/share/code_bot/config/config_bot.json"
        secret_path = os.environ.get("API_KEY_PATH")
        secret_path = "/root/workspace/secrets/api_key.json"

        # Decision context
        self.decision_context_json_path="/root/workspace/contexts/decision_context.json"
        self.decision_context_txt_path="/root/workspace/contexts/decision_context.txt"

        # Evaluation context
        self.evaluation_context_json_path="/root/workspace/contexts/evaluation_context.json"
        self.evaluation_context_txt_path="/root/workspace/contexts/evaluation_context.txt"

        # Correction context
        self.correction_context_json_path="/root/workspace/contexts/correction_context.json"
        self.correction_context_txt_path="/root/workspace/contexts/correction_context.txt"

        # Decision bot
        self.decision_bot = ChatGPT(config_path, secret_path)
        self.code_deployment_client = self.create_client(CodeExecution, 'chat_gpt_bot/test_code', callback_group = service_group)
        while not self.code_deployment_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Service not available, waiting again...')
        self.code_execution_req = CodeExecution.Request()
        self._create_json_from_txt(self.decision_context_txt_path, self.decision_context_json_path)
        context = [self._read_json_to_dict(self.decision_context_json_path)]
        self.decision_bot.set_context(context)

        # Evaluation bot
        self.evaluation_bot = ChatGPT(config_path, secret_path)
        self.evaluation_service = self.create_service(EvaluationCode, 'chat_gpt_bot/evaluation_code', self._evaluation_service_callback, callback_group = service_group)
        self.evaluation_result_sub = self.create_subscription(ResultEvaluation, 'chat_gpt_bot/evaluation_results', self._evaluation_result_callback, 10 )
        self._create_json_from_txt(self.evaluation_context_txt_path, self.evaluation_context_json_path)
        context = [self._read_json_to_dict(self.evaluation_context_json_path)]
        self.evaluation_bot.set_context(context)

        # Correction bot
        self.correction_bot = ChatGPT(config_path, secret_path)
        self.final_states_client = self.create_client(ObjectStatesR, 'chat_gpt/final_states', callback_group = service_group)
        while not self.final_states_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Service not available, waiting again...')
        self.final_states_req = ObjectStatesR.Request()
        self._create_json_from_txt(self.correction_context_txt_path, self.correction_context_json_path)
        context = [self._read_json_to_dict(self.correction_context_json_path)]
        self.correction_bot.set_context(context)

    # Utility functions
    def _read_json_to_dict(self, json_file):
            with open(json_file, 'r') as f:
                data = json.load(f)
            return data
            
    def _create_json_from_txt(self, txt_file, json_file):
        # Open the text file for reading
        with open(txt_file, 'r') as file:
            # Read the content of the text file
            content = file.read().strip()  # Remove any leading/trailing whitespace

        content = content.replace('\n', ' ')
        
        # Construct the JSON object with a fixed role and the content from the text file
        data = {
            "role": "user",
            "content": content
        }
        
        # Open the JSON file for writing
        with open(json_file, 'w') as file:
            # Write the dictionary as a JSON formatted string into the file
            json.dump(data, file, indent=4)

    def _clean_code(self, code: str):
        '''
        Clean the code from the overhead
        '''
        pattern = r"```python(.*?)```"
        matches = re.findall(pattern, code, re.DOTALL)

        if matches:
            return matches[0]
        else:
            return code

    def _stringifyObjects(self, objStates):
        '''
        Return a string that contains the information contained in the dictionary objStates
        '''
        stringified = "{"
        for object, state in zip(objStates.keys(), objStates.values()):
            stringified += f" {object} : [{state[0]}, {state[1]}, {state[2]}] ,"
        stringified += "}"
        return stringified

    # Public interface
    def request_task(self, task):
        # Keep track of the task
        self.task = task

        # Reset stored variables
        self.evaluation_code = None
        self.attempts = 1

        code = self.decision_bot.chat(f"Task: {task}")
        code = self._clean_code(code)

        # ***** DEBUG ******
        print("\n\n")
        print(f"Code #{self.attempts}:\n {code}")
        print("\n\n")
        # ******************

        self._deploy_code(code)

    # Inner workings
    def _deploy_code(self, code: str):
        '''
        Deploys the code in a simulated environment
        '''
        # Keep track of the code
        self.code = code
        # Increment attemps counter
        self.code_execution_req.code = code
        future = self.code_deployment_client.call_async(self.code_execution_req)
        future.add_done_callback(self._code_errors_callback)

    def _code_errors_callback(self, future):
        '''
        Print the errors triggered by code execution
        '''
        print("Code execution generated the following error |{}|".format(future.result().code_except))
        self.code_errors = future.result().code_except
        self.objStates = None
        self._correct_code()

    def _evaluation_service_callback(self, request, response):
        '''
        Manages requests for evaluation code coming from outside
        '''
        if self.evaluation_code:
            evaluation_code = self.evaluation_code
        else:
            evaluation_code = self.evaluation_bot.chat(self.task)
            evaluation_code = self._clean_code(evaluation_code)

        # ***** DEBUG ******
        print("\n\n")
        print(f"Evaluation code #{self.attempts}:\n {evaluation_code}")
        print("\n\n")
        # ******************

        response.evaluation_code = evaluation_code
        return response

    def _evaluation_result_callback(self, msg):
        print("Evaluation results received")
        if msg.completion_flag == True:
            print("The task was succesfully completed")
            print("The succesful code is:\n", self.code)
        else:
            if msg.eval_except != "":
                print("The evaluation generated the following error", msg.eval_except)
            print("Code execution failed to solve the task")
            if self.attempts < self.MAX_ATTEMPTS:
                self.code_errors = ""
                self.objStates = {object:state.pose for object, state in zip(msg.objects,msg.states)}
                self._correct_code()
            else:
                print("Unable to perform the prompted task") 

    def _correct_code(self):
        '''
        Provides feedback on how to correct the code
        '''
        # Analyze code and object states
        if self.objStates:
            correction_report = self.correction_bot.chat(f"Task: {self.task}, Code: {self.code}, Objects: {self._stringifyObjects(self.objStates)}")
        else:
            correction_report = self.correction_bot.chat(f"Task: {self.task}, Code: {self.code}, Code errors: {self.code_errors}")

        # ***** DEBUG ******
        print("\n\n")
        print(f"Retort #{self.attempts}:\n {correction_report}")
        print("\n\n")
        # ******************

        # Reset stored code errors
        self.code_errors = ""

        # Generate new code
        new_code = self.decision_bot.chat(f"Task: {self.task}, Old Code: {self.code}, Report: {correction_report}")

        # Deploy new code
        self._deploy_code(new_code)

def main(args=None):
    rclpy.init(args=args)
    node = CodeNode()
    task = input("Enter the task: ")
    node.request_task(task)
    rclpy.spin(node)
    rclpy.shutdown()