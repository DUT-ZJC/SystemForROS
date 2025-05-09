import math

import rclpy
from rclpy.executors import ExternalShutdownException
import execute_movement_msgs as emm
import execute_movement as emc
from execute_movement.Robot_Act import RobotAct
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

class RobotServer(rclpy.node.Node):
    def __init__(self, namespace=None):
        super().__init__('Surgery_Robot_Server')
        if namespace is None:
            namespace = self.get_namespace()
        if not namespace.endswith('/'):
            namespace = namespace+'/'
        
        self.Stop_Flag = True

        self.declare_parameter('rotation_delta_degrees', 25)
        self.declare_parameter('translation_delta_meters', 0.1)
        self.declare_parameter('max_velocity_scaling', 0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        angle_delta = math.radians(self.get_parameter('rotation_delta_degrees').get_parameter_value().double_value)
        translation_delta = self.get_parameter('translation_delta_meters').get_parameter_value().double_value
        max_velocity_scaling = self.get_parameter('max_velocity_scaling').get_parameter_value().double_value
        max_acceleration_scaling = self.get_parameter('max_acceleration_scaling').get_parameter_value().double_value

        self.local_mover = RobotAct(self, 
                                                max_velocity_scaling=max_velocity_scaling,
                                                max_acceleration_scaling=max_acceleration_scaling,
                                                angle_delta=angle_delta, translation_delta=translation_delta)

        # setup services and topics

        self.move_to_start_service = self.create_service(emm.srv.Move_To_Strat, emc.TOSTART_TOPIC, self.move_to_start)
        self.keep_move_service = self.create_service(emm.srv.Keep_Move, emc.KEEPMOVE_TOPIC, self.keep_move)
        self.stop_move_service = self.create_service(emm.srv.Stop_Move, emc.STOPMOVE_TOPIC, self.stop_move)
        self.Automove_start_service = self.create_service(emm.srv.AutoMove_Start, emc.AUTOMOVE_TOPIC, self.Automove_start)
        self.stop_and_back_service = self.create_service(emm.srv.Stop_And_Back, emc.STOPBACK_TOPIC, self.stop_and_back)

    def move_to_start(self, _):

        State = self.local_mover.set_and_moveto_starting_position()

        return emm.srv.Move_To_StratResponse(success=State)

    def keep_move(self, _):

        State = self.local_mover.keep_moving()

        return emm.srv.Keep_MoveResponse(success=State)

    def stop_move(self, _):

        State = self.local_mover.Stop()
    
        return emm.srv.Stop_MoveResponse(success=State)

    def Automove_start(self, req):

        State = self.local_mover.Automove(req.target_pose_index)

        return emm.srv.Keep_MoveResponse(success=State)

    def stop_and_back(self, _):
        State = self.local_mover.Stop_and_Back()
        return emm.srv.Stop_And_BackResponse(success=State)
    


def main(args=None):
    rclpy.init(args=args)

    executer_robot = RobotServer()

    try:
        # 使用 MultiThreadedExecutor 为每个服务分配一个线程
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(executer_robot)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executer_robot.destroy_node()
        executor.shutdown()



if __name__ == '__main__':
    main()