import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import copy
from robotgpt_interfaces.srv import CodeExecution, EECommands, Trajectory
from robotgpt_interfaces.msg import StateReward, Action, StateEnv, StartM, ResetRequest
from panda_env.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
from cv_bridge import CvBridge
import cv2


class ImitationNode(Node):

    def __init__(self):
        super().__init__('ImitationNode')
        print("Hello from ImitationNode!")

        service_group = ReentrantCallbackGroup()
        topic_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        #subscription to start the imitation
        self.start_sub = self.create_subscription(StartM, 'imitation/start', self.start_callback, 10,  callback_group = service_group)

        #publisher of the action to the env, use for testing and trying the policy
        self.action_pub = self.create_publisher(EECommandsM,'trajectory_execution', 10, callback_group = service_group)
        #subscription to the state (image, in hand image, gripper boolean)
        self.state_sub = self.create_subscription(StateEnv, '/panda_env/stateEnv', self.state_callback, 10, callback_group = topic_group)

        #publisher to reset the env. Needed every time you want to start a simulation
        self.reset_pub = self.create_publisher(ResetRequest, '/panda_env/reset_request', 10)

        self.policy = None

    def start_callback(self, msg):
        #TODO: put here the code for imitation
        #the database is in workspace/database


        #finish training, you should obtain a policy
        #1) reset the panda environment
        msg_reset = ResetRequest()
        self.reset_pub.publish(msg_reset)
        #you should receive a state as a response to the reset,
        # the state will start the action-state response
    
    def state_callback(self, msg):
        height_map = self.bridge.imgmsg_to_cv2(msg.height_image, desired_encoding='8UC1')
        inhand_image = self.bridge.imgmsg_to_cv2(msg.inhand_image, desired_encoding='8UC1')
        gripper_status = msg.gripper
        #give the current state to the policy in order to obtain the action
        action = self.policy(state)
        
        msg = EECommandsM()
        msg.target_state = action[a:b]
        msg.pick_or_place = action[c]
        msg.end_task = False

        self.action_pub.publish(msg)

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

