import rclpy
from rclpy.node import Node
import time

from robotgpt_interfaces.msg import StateReward, Action
from geometry_msgs.msg import Point
import gym_example
import gymnasium
import numpy as np

class PandaEnvROSNode(Node):
    def __init__(self):
        super().__init__('panda_env_node')

        self.env = gymnasium.make('PandaEnv-v0')

        #publisher for environment state
        self.state_pub = self.create_publisher(StateReward, '/panda_env/state', 10)
        print("--------- state pub created -------")


        # Subscriber for agent actions
        self.action_sub = self.create_subscription(Action, 'traj', self.action_callback, 20)
        print("--------- action created -------")

        # Initialize action variable
        self.action = None

        # ROS loop rate (decide if needed)
        # self.rate = rospy.Rate(10)  # 10 Hz

    def action_callback(self, msg):
        print("msg ricevuto")
        print(msg)
        self.action = msg.action
        action = np.array([self.action[0], self.action[1], self.action[2],
                           self.action[3], self.action[4], self.action[5], self.action[6],
                           self.action[7]])

        next_state, reward, done, _, info = self.env.step(action)
        while np.linalg.norm(next_state[0:3] - action[0:3]) > 1e-3:
            next_state, reward, done, _, info = self.env.step(action)
        print("state reached")

        # Publish current state
        keys = list(info)
        info_value = list(info.values())[0]
        state_msg = StateReward(state=next_state, info_keys=keys, info=info_value, reward=0.1, terminal=False)
        print("state ", state_msg)
        self.state_pub.publish(state_msg)

        # Update current state
        state = next_state

    def initialize(self):
        print("Initialising the env .....")
        state, info = self.env.reset()
        keys = list(info)
        info_value = list(info.values())[0]
        state_msg = StateReward(state=state, info_keys=keys, info=info_value, reward=0.1, terminal=False)
        print(state_msg)
        self.state_pub.publish(state_msg)

def main(args=None):
    #prova
    rclpy.init(args=args)
    panda_env_node = PandaEnvROSNode()
    panda_env_node.initialize()
    rclpy.spin(panda_env_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    panda_env_node.env.close()
    panda_env_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
