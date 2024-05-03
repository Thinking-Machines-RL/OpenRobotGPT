import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands
from robotgpt_interfaces.msg import StateReward, Action, State
from robot_api_layer.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
from collections import deque

class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        #implement argument for chosing between use with chatgpt and use with imitation learning

        #create service for current position
        self.planner = PlannerInterface()
        self.cur_state_sub = self.create_subscription(State, '/panda_env/state', self.state_callback, 10)
        self.cur_state = np.array([0.4315144419670105, -5.4939169533141374e-12, 0.2346152812242508, 1, 0, 0, 0, 0])
        self.EPSILON = 1e-3

        service_group = MutuallyExclusiveCallbackGroup()

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

        #chatgpt service
        self.srv = self.create_service(CodeExecution, 'test_code', self.test_callback, callback_group = service_group)
        #trajectory execution service
        self.client_trajectory = self.create_client(EECommands, 'trajectory_execution')

        while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('Trajectory execution service not available, waiting again...')
        self.req = EECommands.Request()

    # def pick_deque(self, object_pose:np.ndarray):
    #     # final pose must be a numpy array of dimension 7 (3+4)
    #     # What I should get is traj + at the end grip aktion
    #     print("obj pos", object_pose )
    #     self.req.target_state = object_pose
    #     self.req.pick_or_place = True
    #     self.req_deque += deque([self.req])
    #     #To avoid deadlock you call client async and obtain a future value
    #     future = self.client_trajectory.call_async(self.req)
    #     #you spin the ros node until the done parameter of future is TRUE
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().accepted_request

    def pick(self, object_pose:np.ndarray):
        print("[INFO] requested pick")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip aktion
        print("obj pos", object_pose )
        self.req.target_state = object_pose
        self.req.pick_or_place = True
        while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            print("service not available")
        #To avoid deadlock you call client async and obtain a future value
        future = self.client_trajectory.call_async(self.req)
        print("future requested")
        #you spin the ros node until the done parameter of future is TRUE
        rclpy.spin_until_future_complete(self, future)
        print("[INFO] future completed")
        # self.client_trajectory.remove_pending_request(future)
        return future.result().completion_flag, future.result().height_map, future.result().in_hand_image, future.result().gripper_state 
     
    
    def place(self, object_pose:np.ndarray):
        print("[INFO] requested place")
        # final pose must be a numpy array of dimension 7 (3+4)
        # What I should get is traj + at the end grip aktion
        self.req.target_state = object_pose
        self.req.pick_or_place = False
        while not self.client_trajectory.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            print("service not available")
        future = self.client_trajectory.call_async(self.req)
        print("future requested")
        #you spin the ros node until the done parameter of future is TRUE
        rclpy.spin_until_future_complete(self, future)
        print("[INFO] future completed")
        return future.result().completion_flag, future.result().height_map, future.result().in_hand_image,  future.result().gripper_state 

    def pick_cube(self):
        # cube position is the 3d position of the cube + 4 components
        # that are the quaternion related to the grip orientation
        # in order to pick the cube
        if self.traj is None:
            cur_state = self.cur_state[0:7]
        else:
            cur_state = self.traj[-1]
        
        traj, vel_traj = self.planner.pick_cube(cur_state)
        self.traj += traj
        self.vel_traj += vel_traj
        print("pick_cube")
        #while np.linalg.norm(self.cur_state - self.traj[-1][:8]) < self.EPSILON:
        #    pass

    def release_cube(self):
        #  cube_position: np array of 7: where and with witch orientation
        #  to release the cube
        if self.traj is None:
            cur_state = self.cur_state[0:7]
        else:
            cur_state = self.traj[-1]

        traj, vel_traj = self.planner.release_cube(cur_state)
        self.traj += traj
        self.vel_traj += vel_traj
        print("release_cube")
        #while np.linalg.norm(self.cur_state - self.traj[-1][:8]) < self.EPSILON:
        #    pass

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


    def test_callback(self, request, response):
        print("received code")
        code = request.code
        print(code)
        evaluation_code = request.evaluation_code

        # Create a dictionary to hold the local scope
        scope = {'self': self}

        except_occurred = False
        completion_flag = False
        code_except = ""
        eval_except = ""

        # Define wrapper functions for your instance methods in the scope
        #scope['move_to'] = lambda pos: self.move_to(pos)
        #scope['pick_cube'] = lambda pos: self.pick_cube(pos)

        # Execute the received code
        try:
            exec(code, globals(), scope)
            if 'execution_func' in scope:
                # Convert the `execution_func` in the scope to a method of self
                from types import MethodType
                execution_func = MethodType(scope['execution_func'], self)
                execution_func()
        except Exception as e:
            except_occurred = True
            code_except = str(e)
            print("Code exception: ", code_except)

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

        response.completion_flag = completion_flag and not except_occurred
        response.code_except = code_except
        response.eval_except = eval_except

        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # ***** DEBUG *****
    # node.pick(np.array([0.6, 0.1, 0.05, 1, 0, 0, 0]))

    # rclpy.spin_once(node)

    # node.place(np.array([0.7, 0.1, 0.05, 1, 0, 0, 0]))
    # *****************

    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

