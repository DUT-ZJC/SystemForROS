import rclpy
import std_srvs
from std_srvs import srv
import execute_movement as emc
import execute_movement_msgs as emm
from execute_movement_msgs import srv
from std_msgs.msg import String

class RobotClient:
    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.msg = String()

        # init services: sampling
        self.node.get_logger().info('Waiting for sampling services')
        self.move_to_start_point = self.node.create_client(emm.srv.Move_To_Strat, emc.TOSTART_TOPIC)
        self.move_to_start_point.wait_for_service()()
        self.keep_move = self.node.create_client(emm.srv.Keep_Move, emc.KEEPMOVE_TOPIC)
        self.keep_move.wait_for_service()
        self.stop_move = self.node.create_client(emm.srv.Stop_Move, emc.STOPMOVE_TOPIC)
        self.stop_move.wait_for_service()
        self.Automove_start = self.node.create_client(emm.srv.AutoMove_Start, emc.AUTOMOVE_TOPIC)
        self.Automove_start.wait_for_service()
        self.stop_and_back = self.node.create_client(emm.srv.Stop_And_Back, emc.STOPBACK_TOPIC)
        self.stop_and_back.wait_for_service()
    

    def MoveToStart(self):
        future = self.move_to_start_point.call_async(emm.srv.Move_To_Strat.Request())
        future.add_done_callback(self.move_to_start_point_callback)
    def move_to_start_point_callback(self,future):
        response = future.result()
        if response == True:
            self.PrintLog("")
        else:
            self.PrintLog("")

    def KeepMove(self):
        future = self.keep_move.call_async(emm.srv.Keep_Move.Request())
        future.add_done_callback(self.keep_move_callback)
    def keep_move_callback(self,future):
        response = future.result()
        if response == True:
            self.PrintLog("")
        else:
            self.PrintLog("")

    def StopMove(self):
        future = self.stop_move.call_async(emm.srv.Stop_Move.Request())
        future.add_done_callback(self.stop_move_callback)
    def stop_move_callback(self,future):
        self.PrintLog("")
    
    def AutoMoveStart(self):
        request = emm.srv.AutoMove_Start.Request()
        request.target_pose_index = self.node.target_index
        future = self.Automove_start.call_async(request)
        future.add_done_callback(self.Automove_start_callback)    
    def Automove_start_callback(self,future):
        response = future.result()
        if response == True:
            self.PrintLog("")
        else:
            self.PrintLog("")

    def StopAndBack(self):
        future = self.stop_and_back.call_async(emm.srv.Stop_And_Back.Request())
        future.add_done_callback(self.stop_and_back_callback)
    def stop_and_back_callback(self,future):
        self.PrintLog("")
        


    def PrintLog(self,Log):
        self.msg.data = Log
        self._node.publish_.publish(self.msg)