import rclpy
from rclpy.node import Node
import time

from robotgpt_interfaces.msg import StateReward, Action, State
from geometry_msgs.msg import Point
import gym_example
import gymnasium
import numpy as np

class PandaEnvROSNode(Node):
    def __init__(self):
        super().__init__('panda_env_node')

        self.env = gymnasium.make('PandaEnv-v0')

        #publisher for environment state
        self.curr_state = None
        self.state_pub = self.create_publisher(State, '/panda_env/state', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("--------- state pub created -------")


        # Subscriber for agent actions
        self.action_sub = self.create_subscription(Action, 'traj', self.action_callback, 20)
        print("--------- action created -------")

        # Initialize action variable
        self.action = None

        # ROS loop rate (decide if needed)
        # self.rate = rospy.Rate(10)  # 10 Hz
    def timer_callback(self):
        if self.curr_state is not None:
            state_msg = State(state=self.curr_state)
            print("state ", state_msg)
            self.state_pub.publish(state_msg)


    def action_callback(self, msg):
        print("msg ricevuto")
        print(msg)
        self.action = msg.action
        action = np.array([self.action[0], self.action[1], self.action[2],
                           self.action[3], self.action[4], self.action[5], self.action[6],
                           self.action[7]])

        next_state, reward, done, _, info = self.env.step(action)
        self.env.render()

        # Publish current state
        keys = list(info)
        info_value = list(info.values())[0]
        self.curr_state = next_state[0:8]
        state_msg = State(state=self.curr_state)
        print("state ", state_msg)
        self.state_pub.publish(state_msg)

        # Update current state
        state = next_state

    def initialize(self):
        print("Initialising the env .....")
        state, info = self.env.reset()
        self.curr_state = state[0:8]
        self.env.render()
        keys = list(info)
        info_value = list(info.values())[0]
        state_msg = State(state=state)
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
