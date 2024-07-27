import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands, Trajectory
from robotgpt_interfaces.msg import StateReward, Action, StateEnv, StartM, ResetRequest, CropM, CropRequest, EECommandsM
from panda_env.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
from cv_bridge import CvBridge
import cv2
import torch

# Add the asrse3_corl20 repository path to the system path
sys.path.append('/root/asrse3_corl20')

# Import the necessary components
from agents.agents_2d.dqn_2d_fcn import DQN2DFCNAgent
from utils.env_wrapper import EnvWrapper
from storage.buffer import ReplayBuffer

class ImitationNode(Node):

    def __init__(self):
        super().__init__('ImitationNode')
        print("Hello from ImitationNode!")

        service_group = ReentrantCallbackGroup()
        topic_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # Subscription to start the imitation
        self.start_sub = self.create_subscription(StartM, 'imitation/start', self.start_callback, 10, callback_group=service_group)

        # Publisher of the action to the env, use for testing and trying the policy
        self.action_pub = self.create_publisher(EECommandsM, 'trajectory_execution', 10, callback_group=service_group)
        
        # Subscription to the state (image, in hand image, gripper boolean)
        self.state_sub = self.create_subscription(StateEnv, '/panda_env/stateEnv', self.state_callback, 10, callback_group=topic_group)

        # Publisher to reset the env. Needed every time you want to start a simulation
        self.reset_pub = self.create_publisher(ResetRequest, '/panda_env/reset_request', 10)

        self.Crop_image_sub = self.create_publisher(CropM, '/panda_env/Crop', self.crop_callback, 1, callback_group=service_group)
        self.Crop_image_request_pub = self.create_subscription(CropRequest, '/panda_env/CropRequest', 10)

        # Hyperparameters
        self.num_epochs = 100  # Number of training epochs
        self.batch_size = 32  # Batch size for training
        self.buffer_size = 10000  # Size of the replay buffer
        self.learning_rate = 0.001  # Learning rate for the optimizer
        self.model_path = '/root/workspace/ros2_ws/src/panda_env/panda_env/trained_agents'  # Path to save/load the trained model        
        self.use_pretrained = False  # Boolean flag to use pretrained model or not

        # Initialize the agent
        self.env_wrapper = EnvWrapper()  # Adjust this to your specific environment wrapper
        self.agent = DQN2DFCNAgent(self.env_wrapper.observation_space, self.env_wrapper.action_space, lr=self.learning_rate)
        self.buffer = ReplayBuffer(buffer_size=self.buffer_size)

        self.current_state = None
        self.expert_data = []  # Placeholder for expert data

        if self.use_pretrained:
            self.load_agent()

    def start_callback(self, msg):
        if not self.use_pretrained:
            # Load expert data from workspace/database
            self.load_expert_data()

            # Train the model with the expert data
            self.train_model()

            # Save the trained model
            self.save_agent()

        # Reset the panda environment to start the simulation
        msg_reset = ResetRequest()
        self.reset_pub.publish(msg_reset)

    def state_callback(self, msg):
        height_map = self.bridge.imgmsg_to_cv2(msg.height_image, desired_encoding='8UC1')
        inhand_image = self.bridge.imgmsg_to_cv2(msg.inhand_image, desired_encoding='8UC1')
        gripper_status = msg.gripper
        
        # Preprocess the state to match the model's input
        state = self.preprocess_state(height_map, inhand_image, gripper_status)
        self.current_state = state

        # Get action from the policy
        action = self.agent.act(state)
        
        # Publish action
        self.publish_action(action)

    def crop_callback(self, msg):
        crop = self.bridge.imgmsg_to_cv2(msg.crop, desired_encoding='8UC1')

        msg = EECommandsM()
        msg.target_state = action[:2]  # Assuming action has 2 components
        msg.pick_or_place = action[2]
        msg.end_task = False

        self.action_pub.publish(msg)

    def load_expert_data(self):
        # Implement this method to load expert data from the database
        self.expert_data = load_data_from_csv('workspace/database/expert_data.csv')
        pass

    def train_model(self):
        # Training loop
        for epoch in range(self.num_epochs):
            for state, action in self.expert_data:
                self.buffer.add(state, action, 0, False)  # Assuming reward is 0 and done is False

            for _ in range(100):  # Adjust number of training steps
                batch = self.buffer.sample(self.batch_size)  # Sample a batch from the buffer
                self.agent.update(batch)

    def preprocess_state(self, height_map, inhand_image, gripper_status):
        # Example preprocessing: Flatten images and concatenate with gripper status
        height_map_flat = height_map.flatten()
        inhand_image_flat = inhand_image.flatten()
        state = np.concatenate((height_map_flat, inhand_image_flat, [gripper_status]), axis=0)
        return state

    def publish_action(self, action):
        msg = EECommandsM()
        msg.target_state = action[:2]  # Adjust according to action dimensions
        msg.pick_or_place = action[2]
        msg.end_task = False
        self.action_pub.publish(msg)

    def save_agent(self):
        torch.save(self.agent.state_dict(), self.model_path)
        self.get_logger().info(f'Model saved to {self.model_path}')

    def load_agent(self):
        if os.path.exists(self.model_path):
            self.agent.load_state_dict(torch.load(self.model_path))
            self.get_logger().info(f'Model loaded from {self.model_path}')
        else:
            self.get_logger().info(f'No pre-trained model found at {self.model_path}')

def main(args=None):
    rclpy.init(args=args)
    node = ImitationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
