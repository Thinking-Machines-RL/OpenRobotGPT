import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
import time

from robotgpt_interfaces.msg import StateReward, Action, State, ObjectStatesRequest, ObjectStates, ObjectPose, TrajCompletionMsg
from robotgpt_interfaces.srv import EECommands, Trajectory
from geometry_msgs.msg import Point

import gym_example
import gymnasium
import numpy as np
import threading
from queue import Queue

class PandaEnvROSNode(Node):
    def __init__(self):
        super().__init__('panda_env_node')

        self.env = gymnasium.make('PandaEnv-v0')

        self.SERVICE_TIMEOUT = 60
        client_cb_group = MutuallyExclusiveCallbackGroup()
        completion_cb_group = MutuallyExclusiveCallbackGroup()
        service_group = ReentrantCallbackGroup()
        timer_group = ReentrantCallbackGroup()
        #publisher for environment state
        self.curr_state = None
        self.state_pub = self.create_publisher(State, '/panda_env/state', 10)
        timer_period = 0.2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, timer_group)
        print("--------- state pub created -------")


        # Client for robot movement
        self.move_client = self.create_client(Trajectory, 'traj', callback_group=client_cb_group)
        while not self.move_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self.get_logger().info('move service not available, waiting again...')
        self.req = Trajectory.Request()
        print("--------- movement sub created -------")

        #Action service
        #Service to be called by the API or the Agent to step the environment
        self.action_service = self.create_service(EECommands, 'trajectory_execution', self.step_callback, callback_group=service_group)

        # Perception
        # Request for object states handler
        self.ObjectStatesRequests = self.create_subscription(ObjectStatesRequest, 'ObjectStatesRequests', self.objReq_callback, 1)
        # Object states pusblisher
        self.ObjectStatesPublisher = self.create_publisher(ObjectStates, '/panda_env/ObjectStates', 1)

        # Trajectory completion
        self.trajCompletionPub = self.create_publisher(TrajCompletionMsg, 'traj_completion', 10, callback_group=completion_cb_group)

        self.executing_trajectory = False
        self.lock = threading.Lock()
        self.request_queue = Queue()
        self.height_map = []

    def objReq_callback(self, req):
        objStates = self.env.getObjStates()
        msg = ObjectStates()
        msg.objects = objStates.keys()
        states = []
        for state in objStates.values():
            op = ObjectPose()
            op.pose = state
            states.append(op)
        msg.states = states
        self.ObjectStatesPublisher.publish(msg)

    def timer_callback(self):
        #timer to publish the current state of the robot end effector
        # print("spinning")
        if self.curr_state is not None:
            state_msg = State(state=self.curr_state)
            self.state_pub.publish(state_msg)
    
    def _traj_generation(self, goal_position, gripper_state):
        request = Trajectory.Request()
        request.current_position = self.curr_state[0:7]
        print("current position ", self.curr_state[0:7])
        request.ending_position = goal_position
        request.gripper_state = gripper_state
        future = self.move_client.call_async(request)
        future.add_done_callback(self._traj_generation_callback)

    def _traj_generation_callback(self, future):
        if future.result() is not None:
            result = future.result()
            position_array = np.array(result.position)
            vel_array = np.array(result.vel)
            traj_pos = np.reshape(position_array, (int(position_array.size/8), 8))
            traj_vel = np.reshape(vel_array, (int(vel_array.size/3), 3))
            traj = np.hstack((traj_pos, traj_vel))
            self._handle_trajectory(result.completion_flag, traj)
            with self.lock:
                self.executing_trajectory = False  # Mark trajectory execution as completed
                # Signal trajectory completion to the API node
                msg = TrajCompletionMsg()
                msg.flag = True
                print("Starting to publish trajectory completion message")
                self.trajCompletionPub.publish(msg)
                print("Trajectory completion message published")
                if not self.request_queue.empty():
                    # Process next request in the queue
                    position, gripper = self.request_queue.get()
                    self._traj_generation(position, gripper)
                    
        else:
            self.get_logger().error('Failed to get trajectory')
            with self.lock:
                self.executing_trajectory = False  # Mark trajectory execution as completed
                # Signal trajectory completion to the API node
                msg = TrajCompletionMsg()
                msg.flag = True
                print("Starting to publish trajectory completion message")
                self.trajCompletionPub.publish(msg)
                print("Trajectory completion message published")

    def _handle_trajectory(self, done, traj):
        print("Starting trajectory")
        print("trajectory size ",traj.shape)
        # curr_state = traj[-1,0:7]
        for step in traj:
            # print("step")
            next_state, _, done, _, _ = self.env.step(step)
            self.env.render()
            self.curr_state = next_state[0:8]
        

    # def _traj_generation(self, goal_position, gripper_state):
    #     print("[INFO] asking for trajectory generation")
    #     self.req.current_position = self.curr_state[0:7]
    #     self.req.ending_position = goal_position
    #     self.req.gripper_state = gripper_state

    #     #To avoid deadlock you call client async and obtain a future value
    #     print("[info] creating future")
    #     future = self.move_client.call_async(self.req)
    #     print("future ", future)
    #     #you spin the ros node until the done parameter of future is TRUE
    #     rclpy.spin_until_future_complete(self, future)
    #     print("spin future completed")
    #     position_array = np.array(future.result().position)
    #     vel_array = np.array(future.result().vel)
    #     traj_pos = np.reshape(position_array, (int(position_array.size/8), 8))
    #     traj_vel = np.reshape(vel_array, (int(vel_array.size/3), 3))
    #     traj = np.hstack((traj_pos, traj_vel))
    #     print("[INFO] trajectory obtained")
    #     flag = future.result().completion_flag
    #     future.cancel()
    #     return flag, traj

    def reset(self):
        print("Initialising the env .....")
        state, info = self.env.reset()
        self.curr_state = state[0:8]
        self.env.render()
        state_msg = State(state=state)
        print(state_msg)
        # self.state_pub.publish(state_msg)
    
    def step_callback(self, request, response):
        position = request.target_state
        gripper = request.pick_or_place
        with self.lock:
            if self.executing_trajectory:
                # Another trajectory is already in progress, queue the request
                self.request_queue.put((position, gripper))
                response.completion_flag = True
                response.height_map = []
                response.in_hand_image = []
                response.gripper_state = gripper
                return response

            # Mark trajectory execution as in progress
            self.executing_trajectory = True
        self._traj_generation(position, gripper)
        response.completion_flag = True
        response.height_map = []
        response.in_hand_image = []
        response.gripper_state = gripper
        return response

    # def step_callback(self, request, response):
    #     print("received action")
    #     position = request.target_state
    #     gripper = request.pick_or_place

    #     print("[INFO] calling traj_generation")
    #     done, traj = self._traj_generation(position, gripper)
    #     # done = True
    #     # traj = np.array([[0.699999988079071, 0.10000000149011612, 0.05000000074505806, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    #     for step in traj:
    #         next_state, reward, done, _, info = self.env.step(step)
    #         self.env.render()
    #         self.curr_state = next_state[0:8]
    #         state_msg = State(state=self.curr_state)
    #         # print("state ", state_msg)
    #         # self.state_pub.publish(state_msg)
    #     done=True
    #     response.completion_flag = done
    #     response.height_map = []
    #     response.in_hand_image = []
    #     response.gripper_state = gripper
    #     print("[INFO] Sent response")

    #     return response

def main(args=None):
    #prova
    rclpy.init(args=args)
    panda_env_node = PandaEnvROSNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(panda_env_node)
    panda_env_node.reset()
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    panda_env_node.env.close()
    panda_env_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
