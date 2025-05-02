import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 定义固定的 TF 变换
        self.transform = TransformStamped()
        self.transform.header.frame_id = "base_frame"
        self.transform.child_frame_id = "effector_frame"
        
        # 设置固定的平移和旋转
        self.transform.transform.translation.x = 1.0  # 示例值
        self.transform.transform.translation.y = 2.0  # 示例值
        self.transform.transform.translation.z = 3.0  # 示例值
        self.transform.transform.rotation.x = 0.0     # 示例值
        self.transform.transform.rotation.y = 0.0     # 示例值
        self.transform.transform.rotation.z = 0.0     # 示例值
        self.transform.transform.rotation.w = 1.0     # 示例值
        
        # 创建定时器，每秒发布一次 TF 变换
        self.timer = self.create_timer(1.0, self.publish_transform)
        self.get_logger().info("TFPublisher Node Started")

    def publish_transform(self):
        # 更新时间戳
        self.transform.header.stamp = self.get_clock().now().to_msg()
        # 发布 TF 变换
        self.tf_broadcaster.sendTransform(self.transform)
        self.get_logger().info("Published TF transform from tr_1 to tr_2")

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()