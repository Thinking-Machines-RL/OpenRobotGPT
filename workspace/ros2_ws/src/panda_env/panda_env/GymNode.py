import rclpy
from rclpy.node import Node

from robotgpt_interfaces.msg import StateReward, Action
from geometry_msgs.msg import Point
import gym_example
import gymnasium

class PandaEnvROSNode(Node):
    def __init__(self):
        super().__init__('panda_env_node')

        self.env = gymnasium.make('PandaEnv-v0')
        #publisher for environment state
        self.state_pub = self.create_publisher(StateReward, '/panda_env/state', 10)

        # Subscriber for agent actions
        self.action_sub = self.create_subscription(Point, 'traj', self.action_callback, 20)

        # Initialize action variable
        self.action = None
        self.run()

        # ROS loop rate (decide if needed)
        # self.rate = rospy.Rate(10)  # 10 Hz

    def action_callback(self, msg):
        print("msg ricevuto")
        print(msg)
        self.action = msg

    def run(self):
        while rclpy.ok():
            # Get current state from the environment
            print("Initialising the env .....")
            state, info = self.env.reset()
            keys = list(info)
            info_value = list(info.values())[0]
            state_msg = StateReward(state=state, info_keys=keys, info=info_value, reward=0.1, terminal=False)
            self.state_pub.publish(state_msg)
            done = False

            while not done:
                if self.action is not None:
                    # Step through the environment
                    # action = self.env.action_space.sample()  # For demonstration, sample random actions
                    action = np.array(self.action.x, self.action.y, self.action.z, 0)
                    next_state, reward, done, _, info = self.env.step(action)

                    # Publish current state
                    keys = list(info)
                    info_value = list(info.values())[0]
                    state_msg = StateReward(state=next_state, info_keys=keys, info=info_value, reward=0.1, terminal=False)
                    self.state_pub.publish(state_msg)

                    # Update current state
                    state = next_state

                    #self.rate.sleep()
                    
                    #reset the action
                    self.action = None
        
        self.env.close()

def main(args=None):
    #prova
    rclpy.init(args=args)
    panda_env_node = PandaEnvROSNode()
    
    rclpy.spin(panda_env_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    panda_env_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
