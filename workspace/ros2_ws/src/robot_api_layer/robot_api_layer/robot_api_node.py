import rclpy
from rclpy.node import Node
import copy
from robotgpt_interfaces.srv import CodeExecution
from robot_api_layer.PlannerInterface import PlannerInterface
import numpy as np
from geometry_msgs.msg import Point
import time
class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        planner = PlannerInterface()
        starting_state = np.zeros(7)
        quat_ending = [1/2, 1/2, 1/2, 1/2]
        starting_state[3:] = starting_state[3:] + quat_ending
        ending_state = np.ones(7)
        ending_state[3:] = ending_state[3:] - quat_ending
        self.traj = planner.plan_trajectory(starting_state, ending_state)
        self.state_pub = self.create_publisher(Point, 'traj', 20)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.srv = self.create_service(CodeExecution, 'test_code', self.test_callback)

    def timer_callback(self):
        point = self.traj[self.i]
        traj_msg = Point(x=point[0], y= point[1], z=point[2])
        print(traj_msg)
        self.state_pub.publish(traj_msg)
        print("[Info] publishing  ", traj_msg)
        self.get_logger().info('Publishing: "%s"' % traj_msg)
        self.i += 1
        if self.i == (self.traj.shape[0]):
            self.i = 0

    def test_callback(self, request, response):
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
        return response
    
    


def main(args=None):
    rclpy.init(args=args)
    node = RobotAPINode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

