import rclpy
from rclpy.node import Node
from robotgpt_interfaces.srv import CodeExecution, EvaluationCode, ObjectStatesR
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from robotgpt_interfaces.msg import ResultEvaluation, CodeExecutionM, CodeError, StartM
from .bots import DecisionBot, CorrectionBot, EvaluationBot, ChatGPT
import pkg_resources
import json
import re
import os
from datetime import datetime
import json
import shutil


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

        #dataset
        self.dataset_path = "/root/workspace/dataset"
        self.current_folder = None
        self.current_traj_folder = None
        self.it = 1
        #list where to store the index of all the failed trajectoeries
        self.failed = []

        # Decision bot
        self.decision_bot = ChatGPT(config_path, secret_path)
        self.code_deployment_publisher = self.create_publisher(CodeExecutionM, 'chat_gpt_bot/test_code', 10)
        self.code_error_subscriber = self.create_subscription(CodeError, 'chat_gpt_bot/code_error', self._code_errors_callback, 10)
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
        self.evaluation_code = None

        # Correction bot
        self.correction_bot = ChatGPT(config_path, secret_path)
        self._create_json_from_txt(self.correction_context_txt_path, self.correction_context_json_path)
        context = [self._read_json_to_dict(self.correction_context_json_path)]
        self.correction_bot.set_context(context)

        self.Imitation_start_pub = self.create_publisher(StartM, 'imitation/start', 10)


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

    def _create_folder(self) -> str:
        '''This function create a folder inside dataset with the name
           of the current time instant in order to differentiate the 
           trials
           
           output:
                str: path of the new folder'''

        #folder name will contain date and time
        current_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        #creation of the folder
        new_folder_name = os.path.join(self.dataset_path, current_datetime)
        os.makedirs(new_folder_name)

        return new_folder_name

    def _create_trajectory_folder(self) -> str:
        '''This function create a folder inside dataset/current_folder 
            with the name ot trajectory_ + the index of the traj 
    
           output:
                str: path of the new folder'''
        #for now only creates trajectory 0
        #TODO add multiple trajectory
        traj_name = "trajectory_" + str(self.it)
        #creation of the folder
        new_folder_name = os.path.join(self.current_folder, traj_name)
        os.makedirs(new_folder_name)
        print("created new traejctroy ", traj_name)

        # Create imgs folder inside the new folder
        os.makedirs(os.path.join(new_folder_name, "imgs"))

        return new_folder_name

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
        '''Request to the LLM to create the code for a certain 
           prompted task. This is the first action of the pipeline'''

        # Keep track of the task, want to use it for the dataset
        self.task = task

        # Reset stored variables
        self.evaluation_code = ""
        self.attempts = 1

        # ***** DEBUG ******
        print("\n\n")
        print(f"Writing CODE...")
        print("\n\n")
        # ******************

        code = self.decision_bot.chat(f"Task: {task}")
        code = self._clean_code(code)

        #Create folder for dataset
        self.current_folder = self._create_folder()
        #save the prompt used to generate the trajectories
        prompt_path = os.path.join(self.current_folder, "prompt.txt")
        # Write the prompt to the prompt.txt file
        with open(prompt_path, 'w') as file:
            file.write(self.task)

        print(f"[INFO] Task prompt saved to: {prompt_path}")

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
        self.it += 1
        self.code = code
        code_execution_req = CodeExecutionM()
        code_execution_req.code = code
                #create trajectory directory:
        self.current_traj_folder = self._create_trajectory_folder()
        self.code_deployment_publisher.publish(code_execution_req)

    def _code_errors_callback(self, msg):
        '''
        Print the errors triggered by code execution
        '''
        if msg.code_except != "":
            print("Code execution generated the following error |{}|".format(msg.code_except))
            self.code_errors = msg.code_except
            self.objStates = None
            print("removing ", self.current_traj_folder)
            # shutil.rmtree(self.current_traj_folder)
            print("removed ")
            self._correct_code()
        else:
            self.objStates = None

    def _evaluation_service_callback(self, request, response):
        '''
        Manages requests for evaluation code coming from outside
        '''

        # ***** DEBUG *****
        print("\n\n")
        print("Writing EVALUATION code ...")
        print("\n\n")
        # *****************


        if self.evaluation_code != "":
            evaluation_code = self.evaluation_code
        else:
            evaluation_code = self.evaluation_bot.chat(self.task)
            evaluation_code = self._clean_code(evaluation_code)
            self.evaluation_code = evaluation_code

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
            if self.it < 10:
                code = self.decision_bot.chat(self.task)
                code = self._clean_code(code)
                self._deploy_code(code)
            if self.it >= 10:
                print(f"failed attempts {self.failed}")
                for idx in self.failed:
                    traj_name = "trajectory_" + str(idx)
                    #creation of the folder
                    new_folder_name = os.path.join(self.current_folder, traj_name)
                    shutil.rmtree(new_folder_name)
                msg = StartM()
                msg.start = True
                self.Imitation_start_pub.publish(msg)
        else:
            if msg.eval_except != "":
                print("The evaluation generated the following error", msg.eval_except)
            if self.attempts < self.MAX_ATTEMPTS and self.it < 10:
                print("Code execution failed to solve the task")
                self.code_errors = ""
                self.objStates = {object:state.pose for object, state in zip(msg.objects,msg.states)}
                self.failed.append(self.it)
                self._correct_code()
            else:
                print("Code execution failed to solve the task")
                print("Unable to perform the prompted task") 

    def _correct_code(self):
        '''
        Provides feedback on how to correct the code
        '''

        # ***** DEBUG ******
        print("\n\n")
        print("Writing REPORT...")
        print("\n\n")
        # ******************

        # Analyze code and object states
        if self.objStates:
            correction_report = self.correction_bot.chat(f"Task: {self.task}, Code: {self.code}, Objects: {self._stringifyObjects(self.objStates)}")
        else:
            correction_report = self.correction_bot.chat(f"Task: {self.task}, Code: {self.code}, Code errors: {self.code_errors}")

        # ***** DEBUG ******
        print("\n\n")
        print(f"Report #{self.attempts}:\n {correction_report}")
        print("\n\n")
        # ******************

        # Reset stored code errors and object states
        self.code_errors = ""
        self.objStates = None

        # ***** DEBUG ******
        print("\n\n")
        print("Writing CORRECTED code...")
        print("\n\n")
        # ******************

        # Generate new code
        new_code = self.decision_bot.chat(f"Task: {self.task}, Old Code: {self.code}, Report: {correction_report}")
        new_code = self._clean_code(new_code)

        # Update attempt number
        self.attempts += 1

        # ***** DEBUG ******
        print("\n\n")
        print(f"Code #{self.attempts}:\n {new_code}")
        print("\n\n")
        # ******************

        # Deploy new code
        self._deploy_code(new_code)

def main(args=None):
    rclpy.init(args=args)
    node = CodeNode()
    task = input("Enter the task: ")
    node.request_task(task)
    rclpy.spin(node)
    rclpy.shutdown()