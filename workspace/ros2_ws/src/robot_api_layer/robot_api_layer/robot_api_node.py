import rclpy
from rclpy.node import Node
import copy
from robotgpt_interfaces.srv import CodeExecution
from robotgpt_interfaces.msg import StateReward, Action, State
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
        self.cur_state_sub = self.create_subscription(State, '/panda_env/state', self.state_callback, 10)
        self.cur_state = np.array([0.4315144419670105, -5.4939169533141374e-12, 0.2346152812242508, 1, 0, 0, 0])

        '''
        #Example of trajectory generation, done by chatgpt
        # API available:
        #     - move_to(stateA)
        #     - pick_cube(stateA)
        #     - release_cube(stateA) 

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

    def move_to(self, final_pose):
        pass

    def get_state(self):
        pass

    def pick_cube(self):
        pass

    def release_cube(self):
        pass

    def timer_callback(self):
        if self.traj is not None:
            if self.i < (self.traj.shape[0]):
                action = self.traj[self.i]
                traj_msg = Action(action=action)
                print(traj_msg)
                self.state_pub.publish(traj_msg)
                print("[Info] publishing  ", traj_msg)
                self.get_logger().info('Publishing: "%s"' % traj_msg)
                self.i += 1

    def state_callback(self, msg):
        self.curr_state =  msg.state


    def test_callback(self, request, response):
        #chat GPT generates code for traj generation
        code = request.code
        evaluation_code = request.evaluation_code

        scope = {}
        
        except_occurred = False
        completion_flag = False
        code_except = ""
        eval_except = ""

        exec(code, globals(), scope)
        try:
            scope['execution_func']()
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

        return response
    
    


def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

