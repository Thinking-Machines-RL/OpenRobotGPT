#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from code_bot.msg import Action, StateReward

class AgentNode:
    def __init__(self):
        rospy.init_node('agent_node', anonymous=True)

        # Publisher for action messages
        self.action_pub = rospy.Publisher('/agent/actions', Action, queue_size=10)

        # Subscriber for state messages
        self.state_sub = rospy.Subscriber('/panda_env/state', StateReward, self.state_callback)

    def state_callback(self, msg):
        # Process state message received from /panda_env/state topic
        
        pass

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Create an instance of your action message type
            action_msg = Action()
            
            # Fill in the action_msg with your desired data

            # Implement logic that decides commands
            x, y, z, fingers = 0, 0, 0, 0

            action_msg.action = [x, y, z, fingers]
            
            # Publish the action message
            self.action_pub.publish(action_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        agent_node = AgentNode()
        agent_node.run()
    except rospy.ROSInterruptException:
        pass
