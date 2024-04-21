import rclpy
from rclpy.node import Node
import copy
from robotgpt_interfaces.srv import CodeExecution
from robotgpt_interfaces.msg import StateReward, Action
from robot_api_layer.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        #implement argument for chosing between use with chatgpt and use with imitation learning

        #create service for current position
        planner = PlannerInterface()
        self.cur_state_sub = self.create_subscription(StateReward, '/panda_env/state', self.starting_state_callback, 10)
        self.starting_state =np.array([0.4315144419670105, -5.4939169533141374e-12, 0.2346152812242508, 1, 0, 0, 0])

        '''
        #Example of trajectory generation, done by chatgpt
        API available:
            - plan_trajectory(stateA, stateB)
            - pick_cube(stateA)
            - release cube(stateA) 

        #test ending point ---
        ending_state = np.array([0.6,0.1,0.05, 1, 0, 0, 0])
        #                  ---

        self.traj = planner.plan_trajectory(self.starting_state, ending_state)
        self.traj = np.vstack((self.traj, planner.pick_cube([0.6,0.1,0.05, 1, 0, 0, 0])))
        self.traj = np.vstack((self.traj, planner.plan_trajectory(ending_state, self.starting_state)))
        '''
        #initialize trajectory list
        self.traj = None
        #publisher for point of the trajectory to follow
        self.state_pub = self.create_publisher(Action, 'traj', 20)
        timer_period = 0.7 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        #chatgpt service
        self.srv = self.create_service(CodeExecution, 'test_code', self.test_callback)

    def timer_callback(self):
        if self.i < (self.traj.shape[0]):
            action = self.traj[self.i]
            traj_msg = Action(action=action)
            print(traj_msg)
            self.state_pub.publish(traj_msg)
            print("[Info] publishing  ", traj_msg)
            self.get_logger().info('Publishing: "%s"' % traj_msg)
            self.i += 1

    def starting_state_callback(self, msg):
        self.starting_state = np.zeros(7)
        self.starting_state[0:3] = msg.state[0:3]


    def test_callback(self, request, response):
        #chat GPT generates code for traj generation
        code = request.code
        evaluation_code = request.evaluation_code

        scope = {}
        
        except_occurred = False
        completion_flag = False
        code_except = None
        eval_except = None
        try:
            exec(code, globals(), scope)
        except Exception as e:
            except_occurred = True
            code_except = str(e)

        exec(evaluation_code, globals(), scope)

        try:
            completion_flag = scope['evaluation_func']()
        except Exception as e:
            except_occurred = True
            eval_except = str(e)


        response.completion_flag = completion_flag and not except_occurred
        response.code_except = code_except
        response.eval_except = eval_except

        #In theory I want to create list of trajectories
        return response
    
    


def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

