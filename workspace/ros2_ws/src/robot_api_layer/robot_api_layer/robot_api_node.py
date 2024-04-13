import rclpy
from rclpy.node import Node
import copy
from robotgpt_interfaces.srv import CodeExecution


class RobotAPINode(Node):

    def __init__(self):
        super().__init__('test_code')
        print("Hello from test_code!")
        self.srv = self.create_service(CodeExecution, 'test_code', self.test_callback)

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

