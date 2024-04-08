#!/usr/bin/env python
import rospy
from env_node.msg import StateReward, Action
import gym_example
import gymnasium

class PandaEnvROSNode:
    def __init__(self):
        rospy.init_node('panda_env_node')
        self.env = gymnasium.make('PandaEnv-v0')

        # Publisher for environment state
        self.state_pub = rospy.Publisher('/panda_env/state', StateReward, queue_size=10)

        # Subscriber for agent actions
        self.action_sub = rospy.Subscriber('/agent/actions', Action, self.action_callback)

        # Initialize action variable
        self.action = None

        # ROS loop rate (decide if needed)
        # self.rate = rospy.Rate(10)  # 10 Hz

    def action_callback(self, msg):
        self.action = msg.data

    def run(self):
        while not rospy.is_shutdown():
            # Get current state from the environment
            state, info = self.env.reset()
            state_msg = StateReward(data=state, info=info, reward=0, terminal=False)
            self.state_pub.publish(state_msg)
            done = False

            while not done:
                if self.action is not None:
                    # Step through the environment
                    #action = self.env.action_space.sample()  # For demonstration, sample random actions
                    next_state, reward, done, _, info = self.env.step(action)

                    # Publish current state
                    state_msg = StateReward(data=next_state, info=info, reward=0, terminal=done)
                    self.state_pub.publish(state_msg)

                    # Update current state
                    state = next_state

                    #self.rate.sleep()
                    
                    #reset the action
                    self.action = None

if __name__ == '__main__':
    try:
        panda_env_node = PandaEnvROSNode()
        panda_env_node.run()
    except rospy.ROSInterruptException:
        pass
