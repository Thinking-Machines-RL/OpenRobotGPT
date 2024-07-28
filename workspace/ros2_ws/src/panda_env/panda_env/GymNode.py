import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
import time

from robotgpt_interfaces.msg import StateReward, Action, State, ObjectStatesRequest, ObjectStates, ObjectPose, TrajCompletionMsg, EECommandsM, ResetRequest, StateEnv, CropM, CropRequest
from robotgpt_interfaces.srv import EECommands, Trajectory, ObjectStatesR
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

import csv
import os
import gym_example
import gymnasium
import numpy as np
import threading
from queue import Queue
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge

class PandaEnvROSNode(Node):
    def __init__(self):
        super().__init__('panda_env_node')

        self.env = gymnasium.make('PandaEnv-v0')
        self.bridge = CvBridge()
        
        self.SERVICE_TIMEOUT = 60
        client_cb_group = MutuallyExclusiveCallbackGroup()
        completion_cb_group = MutuallyExclusiveCallbackGroup()
        service_group = ReentrantCallbackGroup()
        timer_group = ReentrantCallbackGroup()

        #publisher for environment state
        self.curr_state = None
        self.state_pub = self.create_publisher(State, '/panda_env/state', 10)
        self.stateEnv_pub = self.create_publisher(StateEnv, '/panda_env/stateEnv', 10)
        timer_period = 1 # seconds
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
        # self.action_service = self.create_service(EECommands, 'trajectory_execution', self.step_callback, callback_group=service_group)
        self.action_sub = self.create_subscription(EECommandsM, 'trajectory_execution', self.step_callback, 10, callback_group=service_group)
        self.initial_object_states_pub = self.create_publisher(ObjectStates, '/panda_env/InitialObjectStates', 10, callback_group=service_group)
        self.reset_sub = self.create_subscription(ResetRequest, '/panda_env/reset_request', self.reset_callback, 10)


        # Perception
        # Object states pusblisher
        self.ObjectStatesPublisher = self.create_publisher(ObjectStates, '/panda_env/ObjectStatesEval', 1)
        self.Crop_image_pub = self.create_publisher(CropM, '/panda_env/Crop', 1)
        self.Crop_image_request_sub = self.create_subscription(CropRequest, '/panda_env/CropRequest', self.crop_callback, 10, callback_group=service_group)

        # Trajectory completion
        self.trajCompletionPub = self.create_publisher(TrajCompletionMsg, 'traj_completion', 10, callback_group=completion_cb_group)

        self.executing_trajectory = False
        self.end_task = False
        self.lock = threading.Lock()
        self.request_queue = Queue()
        self.height_map = []
        self.action_counter = 0
        self.last_action_pick = False

    def reset_callback(self, msg):
        with self.lock:
            print("[info]: reset call")
            self.reset()
        self.action_counter = 0
        objStates = self.env.getObjStates()

        initObj = ObjectStates()
        initObj.objects = objStates.keys()
        states = []
        for state in objStates.values():
            op = ObjectPose()
            op.pose = state
            states.append(op)
        initObj.states = states
        self.initial_object_states_pub.publish(initObj)


    def timer_callback(self):
        #timer to publish the current state of the robot end effector
        # print("spinning")
        if self.curr_state is not None:
            print("self.curr_state = ", self.curr_state)
            state_msg = State(state=self.curr_state)
            self.state_pub.publish(state_msg)
    
    def _traj_generation(self, goal_position, gripper_state, end_task):
        request = Trajectory.Request()
        print("[INFO] endt task = ", end_task)
        self.end_task = end_task
        request.current_position = self.curr_state[0:7]
        print("current position ", self.curr_state[0:7])
        request.ending_position = goal_position
        request.gripper_state = gripper_state

        #add action to database
        dataset_path = "/root/workspace/dataset"
        current_folder_date = self._retrieve_last_folder(dataset_path)
        print("recent path ", current_folder_date)
        current_folder_traj = self._retrieve_last_folder(current_folder_date)
        print("current folder traj ", current_folder_traj)

        self._add_action_csv(current_folder_traj, goal_position.tolist(), gripper_state)

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
            # Signal trajectory completion to the API node
            msg = TrajCompletionMsg()
            msg.flag = True
            print("Starting to publish trajectory completion message")
            self.trajCompletionPub.publish(msg)
            print("Trajectory completion message published")
            with self.lock:
                if not self.request_queue.empty():
                    self.executing_trajectory = False  # Mark trajectory execution as completed
                    # Process next request in the queue
                    position, gripper, end_task = self.request_queue.get()    
                    self._traj_generation(position, gripper, end_task)
                    
        else:
            self.get_logger().error('Failed to get trajectory')
            with self.lock:
                self.executing_trajectory = False  # Mark trajectory execution as completed
                # Signal trajectory completion to the API node

    def _handle_trajectory(self, done, traj):
        print("Starting trajectory")
        # curr_state = traj[-1,0:7]

        if self.end_task:
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
            print("[INFO] published message")
            print(msg)
        else:
            dataset_path = "/root/workspace/dataset"
            current_folder_date = self._retrieve_last_folder(dataset_path)
            current_folder_traj = self._retrieve_last_folder(current_folder_date)

            # State
            height_map, in_hand_img = self.env.render_images(traj[0,:])

            vmin = 0
            vmax = 0.75 - 0.5

            # Save height_map
            plt.imshow(height_map, vmin=vmin, vmax=vmax)
            plt.axis("off")
            plt.savefig(os.path.join(current_folder_traj, "imgs", f"height_map_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)

            # Save in_hand_img
            if self.last_action_pick:
                plt.imshow(in_hand_img, vmin=vmin, vmax=vmax)
                plt.axis("off")
                plt.savefig(os.path.join(current_folder_traj, "imgs", f"in_hand_img_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)
            else:
                plt.imshow(np.zeros((200,200)), vmin=vmin, vmax=vmax)
                plt.axis("off")
                plt.savefig(os.path.join(current_folder_traj, "imgs", f"in_hand_img_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)

            # Write states.csv
            state = [self.action_counter, self.action_counter, True if traj[0,7] <= 0.02 else False]
            self._add_state_csv(current_folder_traj, state)

            for i, step in enumerate(traj):   
                with self.lock:
                    next_state, _, done, _, _ = self.env.step(step)
                    rgb, depth = self.env.render()
                    self.curr_state = next_state[0]
                    state_to_be_saved = next_state[-3:]
                    if step[7] <= 0.02:
                        self.last_action_pick = True
                    else:
                        self.last_action_pick = False
            
            # Next state
            next_height_map, next_in_hand_img = self.env.render_images(traj[traj.shape[0]-1,:])

            # Save height_map
            plt.imshow(next_height_map, vmin=vmin, vmax=vmax)
            plt.axis("off")
            plt.savefig(os.path.join(current_folder_traj, "imgs", f"next_height_map_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)

            # Save in_hand_img
            if self.last_action_pick:
                plt.imshow(next_in_hand_img, vmin=vmin, vmax=vmax)
                plt.axis("off")
                plt.savefig(os.path.join(current_folder_traj, "imgs", f"next_in_hand_img_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)
            else:
                plt.imshow(np.zeros((200,200)), vmin=vmin, vmax=vmax)
                plt.axis("off")
                plt.savefig(os.path.join(current_folder_traj, "imgs", f"next_in_hand_img_{self.action_counter}.png"), bbox_inches='tight', pad_inches=0)

            # Write states.csv
            next_state = [self.action_counter, self.action_counter, True if traj[0,7] <= 0.02 else False]
            self._add_next_state_csv(current_folder_traj, next_state)
            
            # Publish state
            msg = StateEnv()
            height_image = cv2.imread(os.path.join(current_folder_traj, "imgs", f"height_map_{self.action_counter}.png"), cv2.IMREAD_GRAYSCALE)
            height_image_msg = self.bridge.cv2_to_imgmsg(height_image, encoding='8UC1')
            msg.height_image = height_image_msg
            in_hand_image = cv2.imread(os.path.join(current_folder_traj, "imgs", f"in_hand_img_{self.action_counter}.png"),cv2.IMREAD_GRAYSCALE)
            in_hand_image_msg = self.bridge.cv2_to_imgmsg(in_hand_image, encoding='8UC1')
            msg.inhand_image = in_hand_image_msg
            grip = self.curr_state[7] < 0.04
            print(f"gripper {grip}")
            msg.gripper = bool(grip)
            self.stateEnv_pub.publish(msg)
            # Update action counter
            self.action_counter += 1
    
    def _retrieve_last_folder(self, main_directory):

        #create list with all directories
        # print("directories")
        # print(os.listdir(main_directory))

        directories = [d for d in os.listdir(main_directory) if os.path.isdir(os.path.join(main_directory, d))]
        # Sort the directories based on their names (which are dates)
        sorted_directories = sorted(directories, reverse=True)
        # print(sorted_directories)
        if sorted_directories:
            # Retrieve the most recent directory (the first one in the sorted list)
            last_folder = sorted_directories[0]
            last_folder_path = os.path.join(main_directory, last_folder)
            # print(f"Last folder retireved: {last_folder_path}")
            return last_folder_path
        else:
            print("No folders found in the dataset directory.")

    def _edit_csv(self, path, state, action, reward, next_state):
        '''
        Save (s,a,r,s') into a new row of the dataset
        s  <-- (height map, in hand image, gripper)
        a  <-- (x, y, theta, gripper)
        r  <-- 1/0
        s' <-- (height map, in hand image, gripper)
        '''

        csv_action_path = os.path.join(path, 'transitions.csv')
        file_exists = os.path.isfile(csv_action_path)

        state_headers = ["height_map", "in_hand_img", "gripper"]
        action_headers = ["x", "y", "theta", "gripper"]
        reward_headers = ["reward"]
        next_state_headers = state_headers

        headers = state_headers + action_headers + reward_headers + next_state_headers

        transition = state + action + reward + next_state

        # Open the CSV file in append mode and write the data
        with open(csv_action_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            # If the file doesn't exist, write the header
            if not file_exists:
                csv_writer.writerow(headers)  # Write header

            csv_writer.writerow(transition)


    def _add_state_csv(self, path, state):
        # state is expected to look like this: [dpt_img, in_hand_img, gripper]
        # where dpt_img, in_hand_img are images and gripper is a boolean (True if closed, False otherwise)
        
        # We write the opposite of the state, since images the state is written when the action is already terminated
        _state = [f"heigh_map_{state[0]}.png", f"in_hand_img_{state[1]}.png", state[2]]

        #path to data.cvs
        csv_states_path = os.path.join(path, 'states.csv')
        file_exists = os.path.isfile(csv_states_path)

        headers = ["height_map", "in_hand_img", "gripper"]

        # Open the CSV file in append mode and write the data
        with open(csv_states_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            # If the file doesn't exist, write the header
            if not file_exists:
                csv_writer.writerow(headers)  # Write header

            # Write the data rows
            csv_writer.writerow(_state)
        # print(f"state appended to: {csv_states_path}")
            
    def _add_next_state_csv(self, path, state):
        # state is expected to look like this: [dpt_img, in_hand_img, gripper]
        # where dpt_img, in_hand_img are images and gripper is a boolean (True if closed, False otherwise)
        
        # We write the opposite of the state, since images the state is written when the action is already terminated
        _state = [f"heigh_map_{self.action_counter}.png", f"in_hand_img_{self.action_counter}.png", not state[2]]

        #path to data.cvs
        csv_states_path = os.path.join(path, 'next_states.csv')
        file_exists = os.path.isfile(csv_states_path)

        headers = ["height_map", "in_hand_img", "gripper"]

        # Open the CSV file in append mode and write the data
        with open(csv_states_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            # If the file doesn't exist, write the header
            if not file_exists:
                csv_writer.writerow(headers)  # Write header

            # Write the data rows
            csv_writer.writerow(_state)
        # print(f"state appended to: {csv_states_path}")

    def _add_action_csv(self, path, state, gripper_state):
        #path to data.cvs
        csv_action_path = os.path.join(path, 'action.csv')
        file_exists = os.path.isfile(csv_action_path)

        headers = ["x", "y", "z", "q0", "q1", "q2", "q3", "gripper_state"]
        # Open the CSV file in append mode and write the data
        with open(csv_action_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            # If the file doesn't exist, write the header
            if not file_exists:
                csv_writer.writerow(headers)  # Write header

            csv_writer.writerow(state + [gripper_state])
        # print(f"state appended to: {csv_action_path}")
            

    def reset(self):
        print("Initialising the env .....")
        self.request_queue = Queue()
        state, info = self.env.reset()
        self.curr_state = state[0:8]
        self.env.render()
        state_msg = State(state=state)
        print(state_msg)
        msg = StateEnv()
        height_image_msg = self.bridge.cv2_to_imgmsg(self.env.Height_map, encoding='32FC1')
        msg.height_image = height_image_msg
        in_hand_image = np.zeros((200,200))
        in_hand_image_msg = self.bridge.cv2_to_imgmsg(in_hand_image, encoding='64FC1')
        msg.inhand_image = in_hand_image_msg
        msg.gripper = False
        self.stateEnv_pub.publish(msg)
        # self.state_pub.publish(state_msg)
    
    def step_callback(self, msg):
        position = msg.target_state
        gripper = msg.pick_or_place
        end_task = msg.end_task
        with self.lock:
            if self.executing_trajectory:
                # Another trajectory is already in progress, queue the request
                self.request_queue.put((position, gripper, end_task))
                return True

            # Mark trajectory execution as in progress
            self.executing_trajectory = True
            self._traj_generation(position, gripper, end_task)
        # response.completion_flag = True
        return True
    
    def crop_callback(self, msg):
        x = msg.x
        y = msg.y
        crop_image = self.env.get_in_hand_image(np.array([x, y, 0.05, 0, 0, 0, 1, 0]))
        msg = CropM()
        crop_msg = self.bridge.cv2_to_imgmsg(crop_image, encoding='32FC1')
        msg.crop = crop_msg
        self.Crop_image_pub.publish(msg)

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
