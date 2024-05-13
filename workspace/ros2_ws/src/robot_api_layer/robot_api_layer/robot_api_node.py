import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands, EvaluationCode, ObjectStatesR
from robotgpt_interfaces.msg import StateReward, Action, State, ObjectStates, ResultEvaluation, EECommandsM, CodeExecutionM, ObjectPose
from robot_api_layer.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
from collections import deque
import threading

class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        #implement argument for chosing between use with chatgpt and use with imitation learning

        service_group = ReentrantCallbackGroup()
        topic_group = ReentrantCallbackGroup()

        #create service for current position
        self.planner = PlannerInterface()
        self.cur_state_sub = self.create_subscription(State, '/panda_env/state', self.state_callback, 10, callback_group = topic_group)
        self.cur_state = np.array([0.4315144419670105, -5.4939169533141374e-12, 0.2346152812242508, 1, 0, 0, 0, 0])
        self.EPSILON = 1e-3

        '''
        #Example of trajectory generation, done by chatgpt
        # API available:
        #     - pick(stateA)
        #     - place(stateA)

        #test ending point ---
        ending_state = np.array([0.6,0.1,0.05, 1, 0, 0, 0])
        #                  ---

        self.traj = planner.plan_trajectory(self.starting_state, ending_state)
        self.traj = np.vstack((self.traj, planner.pick_cube([0.6,0.1,0.05, 1, 0, 0, 0])))
        self.traj = np.vstack((self.traj, planner.plan_trajectory(ending_state, self.starting_state)))
        '''
        self.SERVICE_TIMEOUT = 60

        # Perception
        # Object states subscriber
        self.objStatesSubscriber = self.create_subscription(ObjectStates, '/panda_env/ObjectStates', self.objectStates_callback, 1, callback_group = service_group)
        self.objects_states = self.create_client(ObjectStatesR, '/panda_env/InitialObjectStates')
        while not self.objects_states.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('objects_states service not available, waiting again...')
        self.req_states = ObjectStatesR.Request()
        self.ObjectsStatesPublisher = self.create_publisher(ObjectStates, '/panda_env/objects_states', 1)

        self.getInitialObjectStates()

        #CHATGPT client and service -----
        self.srv = self.create_service(CodeExecution, 'chat_gpt_bot/test_code', self.test_callback, callback_group = service_group)
        # self.sub_exec = self.create_subscription(CodeExecutionM, 'chat_gpt_bot/test_code', self.test_callback,10, callback_group = service_group)
        self.client_evaluation_code = self.create_client(EvaluationCode, 'chat_gpt_bot/evaluation_code', callback_group = service_group)
        while not self.client_evaluation_code.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Evaluation code service not available, waiting again...')
        self.req_eval = EvaluationCode.Request()
        self.eval_publisher = self.create_publisher(ResultEvaluation, 'chat_gpt_bot/evaluation_results', 10)
        #--------------------------------

        # Final states server
        self.states_server = self.create_service(ObjectStatesR, 'chat_gpt/final_states', self.final_states_callback, callback_group = service_group)
        # -------------------

        #trajectory execution service
        # self.client_trajectory = self.create_client(EECommands, 'trajectory_execution', callback_group = service_group)
        self.traj_pub = self.create_publisher(EECommandsM,'trajectory_execution', 10, callback_group = service_group)
        # while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
        #     self.get_logger().info('Trajectory execution service not available, waiting again...')
        # self.req = EECommands.Request()
        print("Node ready")

    def pick(self, object_pose:np.ndarray, end_task = False):
        print("[INFO] requested pick")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip aktion
        print("obj pos", object_pose )
        msg = EECommandsM()
        msg.target_state = object_pose
        msg.pick_or_place = True
        msg.end_task = end_task

        self.traj_pub.publish(msg)

        # Mark trajectory as in execution
        self.execution = True

        # while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
        #     print("service not available")
        # #To avoid deadlock you call client async and obtain a future value
        # future = self.client_trajectory.call_async(self.req)
        # print("future requested")
        # #you spin the ros node until the done parameter of future is TRUE
        # rclpy.spin_until_future_complete(self, future)
        # print("[INFO] future completed")

        # self.client_trajectory.remove_pending_request(future)
        # return future.result().completion_flag
     
    
    def place(self, object_pose:np.ndarray, end_task = False):
        print("[INFO] requested place")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip aktion
        msg = EECommandsM()
        msg.target_state = object_pose
        msg.pick_or_place = True
        msg.end_task = end_task

        self.traj_pub.publish(msg)

        # while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
        #     print("service not available")
        # future = self.client_trajectory.call_async(self.req)
        # print("future requested")
        # #you spin the ros node until the done parameter of future is TRUE
        # rclpy.spin_until_future_complete(self, future)
        # print("[INFO] future completed")

        # return future.result().completion_flag
    
    def getInitialObjectStates(self):
        print("[INFO] requested initial state objects")
        future = self.objects_states.call_async(self.req_states)
        rclpy.spin_until_future_complete(self, future)
        objects =  future.result().objects
        states =  future.result().states
        states = [states[i].pose for i in range(len(states))]
        objStates = {object:list(state) for object,state in zip(objects, states)}
        self.objStates = objStates

    # Object states
    def objectStates_callback(self, msg):
        print("[INFO] received object end task")
        objects = msg.objects
        states = msg.states
        states = [states[i].pose for i in range(len(states))]
        self.objStates = {object:list(state) for object,state in zip(objects, states)}

        future = self.client_evaluation_code.call_async(self.req_eval)
        future.add_done_callback(self.code_eval_callback)
    
    def code_eval_callback(self, future):
        evaluation_code =  future.result().evaluation_code
        print(evaluation_code)
        # Create a dictionary to hold the local scope
        scope = {'self': self}

        except_occurred = False
        completion_flag = False
        eval_except = ""

        # Define wrapper functions for your instance methods in the scope

        # Execute the evaluation code
        try:
            exec(evaluation_code, globals(), scope)
            if 'evaluation_func' in scope:
                evaluation_func = MethodType(scope['evaluation_func'], self)
                completion_flag = evaluation_func()
        except Exception as e:
            except_occurred = True
            eval_except = str(e)
            print("Eval exception: ", eval_except)

        # Fill message with data on the evaluation and final object states
        msg = ResultEvaluation()
        msg.completion_flag = completion_flag and not except_occurred
        msg.eval_except = eval_except
        msg.objects = self.objStates.keys()
        states = []
        for state in self.objStates.values():
            p = ObjectPose()
            p.pose = state
            states.append(p)
        msg.states = states
        self.eval_publisher.publish(msg)

        print("Published message end task")
        print(msg)



    def send_request(self, ):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        if self.traj is not None:
            if len(self.traj) > 0:

                action = self.traj[0]
                self.traj.popleft()

                vel_action = self.vel_traj[0]
                self.vel_traj.popleft()
                print("vel action", vel_action)
                print("action", action)
                traj_msg = Action(action=action, vel=vel_action)
                print(traj_msg)
                self.state_pub.publish(traj_msg)
                print("[Info] publishing  ", traj_msg)
                self.get_logger().info('Publishing: "%s"' % traj_msg)

    def state_callback(self, msg):
        self.curr_state =  msg.state
        print("state")

    def execute_code(self, code, scope):
        # try:
        #     exec(code, globals(), scope)
        #     if 'execution_func' in scope:
        #         # Convert the `execution_func` in the scope to a method of self
        #         from types import MethodType
        #         execution_func = MethodType(scope['execution_func'], scope['self'])
        #         execution_func()
        # except Exception as e:
        #     print("Code exception: ", e)
        objStates = self.getInitialObjectStates()
        blue_cube_pos = objStates["blue_cube"]
        # self.pick(blue_cube_pos + [1, 0, 0, 0])
        # self.place([0.7, 0.1, 0.05, 1, 0, 0, 0], True)


    def test_callback(self, request, response):
        print("received code")
        code = request.code
        print(code)

        # Create a dictionary to hold the local scope
        scope = {'self': self}

        try:
            exec(code, globals(), scope)
            if 'execution_func' in scope:
                # Convert the `execution_func` in the scope to a method of self
                from types import MethodType
                execution_func = MethodType(scope['execution_func'], self)
                execution_func()
            code_except = " "
        except Exception as e:
            except_occurred = True
            code_except = str(e)
            print("Code exception: ", code_except)

        response.code_except = code_except
        return response

    

def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

