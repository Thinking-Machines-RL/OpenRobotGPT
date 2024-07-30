import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands, EvaluationCode, ObjectStatesR
from robotgpt_interfaces.msg import StateReward, Action, State, ObjectStates, ResultEvaluation, EECommandsM, ObjectPose, ObjectStatesRequest, CodeExecutionM, CodeError, ResetRequest
from robot_api_layer.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
from collections import deque
import threading
from types import MethodType
import copy
from math import sin, cos

class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")

        service_group = ReentrantCallbackGroup()
        topic_group = ReentrantCallbackGroup()

        #create service for current position
        self.planner = PlannerInterface()
        self.cur_state_sub = self.create_subscription(State, '/panda_env/state', self.state_callback, 10, callback_group = topic_group)
        self.cur_state = np.array([0.4315144419670105, -5.4939169533141374e-12, 0.2346152812242508, 1, 0, 0, 0, 0])
        self.EPSILON = 1e-3

        self.SERVICE_TIMEOUT = 60

        # Perception
        # Object states subscriber
        self.objStatesEvalSubscriber = self.create_subscription(ObjectStates, '/panda_env/ObjectStatesEval', self.objectStatesEval_callback, 1, callback_group = service_group)
        self.objects_states_req = self.create_subscription(ObjectStates, '/panda_env/InitialObjectStates', self.code_execution_callback, 10)
        self.reset_pub = self.create_publisher(ResetRequest, '/panda_env/reset_request', 10)
        self.ObjectsStatesPublisher = self.create_publisher(ObjectStates, '/panda_env/objects_states', 1)
        self.req_states = ObjectStatesR.Request()

        self.pickedObject = None

        #CHATGPT client and service -----
        self.code_subscriber = self.create_subscription(CodeExecutionM, 'chat_gpt_bot/test_code', self.task_callback, 10)
        # self.sub_exec = self.create_subscription(CodeExecutionM, 'chat_gpt_bot/test_code', self.test_callback,10, callback_group = service_group)
        self.client_evaluation_code = self.create_client(EvaluationCode, 'chat_gpt_bot/evaluation_code', callback_group = service_group)
        while not self.client_evaluation_code.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Evaluation code service not available, waiting again...')
        self.req_eval = EvaluationCode.Request()
        self.eval_publisher = self.create_publisher(ResultEvaluation, 'chat_gpt_bot/evaluation_results', 10)
        self.code_execution_resp_publisher = self.create_publisher(CodeError, 'chat_gpt_bot/code_error', 10)
        #--------------------------------

        self.traj_pub = self.create_publisher(EECommandsM,'trajectory_execution', 10, callback_group = service_group)
        print("Node ready")

    def pick(self, object_pose):

        assert isinstance(object_pose, list), "The pose (first argument of pick) should be a list"
        assert all(isinstance(item, float) or isinstance(item, int) for item in object_pose), "All elements of the pose should be floats"
        assert len(object_pose) == 7, "The pose has wrong length. It should be 7 elements long"

        print("[INFO] requested pick")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip action
        msg = EECommandsM()
        msg.target_state = object_pose
        msg.pick_or_place = True
        msg.end_task = False

        self.traj_pub.publish(msg)

        # Mark trajectory as in execution
        self.execution = True

    
    def place(self, object_pose):

        assert isinstance(object_pose, list), "The pose (first argument of place) should be a list"
        assert all(isinstance(item, float) or isinstance(item, int) for item in object_pose), "All elements of the pose should be floats"
        assert len(object_pose) == 7, "The pose has wrong length. It should be 7 elements long"

        print("[INFO] requested place")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip aktion
        msg = EECommandsM()
        msg.target_state = object_pose
        msg.pick_or_place = False
        msg.end_task = False

        self.traj_pub.publish(msg)

        # Mark trajectory as in execution
        self.execution = True


    def end_task_command(self):
        ''' Terminate task '''
        msg = EECommandsM()
        msg.target_state = [0,0,0,1,0,0,0] # fake target state
        msg.pick_or_place = False
        msg.end_task = True

        self.traj_pub.publish(msg)


    def pickUp(self, object):
        ''' Pick up the specified object '''
        assert object in self.objStates.keys(), f"pickUp({object}): '{object}' is not an object."
        assert not self.pickedObject, f"You already picked the object {self.pickedObject}, but you didn't place it"
        PICK_POSE = self.objStates[object]
        self.objStates.pop(object)
        self.pickedObject = object
        self.pick(PICK_POSE)


    def placeObjectOn(self, object, dx=0, dy=0):
        ''' Place the object that we have grasped on top of the specified object '''
        assert self.pickedObject, "placeObjectOn({object}): No object has been picked yet."
        assert object in self.objStates.keys(), f"placeOnObject({object}): '{object}' is not an object."
        BLOCK_HEIGHT = 0.05
        PLACE_POSE = copy.deepcopy(self.objStates[object])
        PLACE_POSE[0] += dx
        PLACE_POSE[1] += dy
        PLACE_POSE[2] += BLOCK_HEIGHT
        self.place(PLACE_POSE)
        self.objStates[self.pickedObject] = PLACE_POSE
        self.pickedObject = None


    def placeInPosition(self, target_position):
        ''' Place the object that we have grasped in the specified position '''
        assert self.pickedObject, "placeInPosition({object}): No object has been picked yet."
        # We choose default orientation [1,0,0,0]
        PLACE_POSE = target_position + [1, 0, 0, 0]
        self.place(PLACE_POSE)
        self.objStates[self.pickedObject] = target_position
        self.pickedObject = None

    def placeSafe(self):
        ''' Place the object in a safe position '''
        assert self.pickedObject, "placeInPosition({object}): No object has been picked yet."
        # We choose default orientation [1,0,0,0]
        target_position = [0.7, -0.1, 0.05]
        PLACE_POSE = target_position + [1, 0, 0, 0]
        self.place(PLACE_POSE)
        self.objStates[self.pickedObject] = PLACE_POSE
        self.pickedObject = None


    def isOnTop(self, object_A, object_B, dx=0, dy=0):
        ''' Checks whether an object A is on top of an object B '''
        BLOCK_DIM = 0.05
        TOLERANCE = 0.01
        position_A = self.objStates[object_A]
        position_B = self.objStates[object_B]
        if abs(position_A[0] - position_B[0] - dx) < TOLERANCE and \
           abs(position_A[1] - position_B[1] - dy) < TOLERANCE and \
           abs(position_A[2] - (position_B[2] + BLOCK_DIM)) < TOLERANCE:
            return True
        return False
    
    def isInPosition(self, object, position):
        ''' Checks whether the object is in the specified position '''
        objPosition = self.objStates[object]
        TOLERANCE = 0.01
        if abs(objPosition[0] - position[0]) < TOLERANCE and \
           abs(objPosition[1] - position[1]) < TOLERANCE and \
           abs(objPosition[2] - position[2]) < TOLERANCE:
            return True
        return False


    # Object states
    def objectStatesEval_callback(self, msg):
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


    def task_callback(self, msg):
        print("received code")
        self.code = msg.code
        print(self.code)
        msg_reset = ResetRequest()
        self.reset_pub.publish(msg_reset)
        print("[INFO] requested initial state objects")

    
    def code_execution_callback(self, msg):

        objects =  msg.objects
        states =  msg.states
        states = [states[i].pose for i in range(len(states))]
        objStates = {object:list(state) for object,state in zip(objects, states)}
        self.objStates = objStates

        print("objStates = ", self.objStates)

        self.pickedObject = None

        # Create a dictionary to hold the local scope
        scope = {'self': self}

        try:
            exec(self.code, globals(), scope)
            if 'execution_func' in scope:
                # Convert the `execution_func` in the scope to a method of self
                execution_func = MethodType(scope['execution_func'], self)
                execution_func()
            code_except = ""
        except Exception as e:
            except_occurred = True
            code_except = str(e)
            print("Code exception: ", code_except)

        self.end_task_command()

        msg_error = CodeError()
        msg_error.code_except = code_except
        self.code_execution_resp_publisher.publish(msg_error)

    

def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

