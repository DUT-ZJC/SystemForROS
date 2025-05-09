from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 引入 LBRDescriptionMixin，用于获取机器人描述相关参数
from lbr_bringup.description import LBRDescriptionMixin
from easy_handeye2.common_launch import arg_calibration_type, arg_tracking_base_frame, arg_tracking_marker_frame

def hidden_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    # robot_description 已由 LBRDescriptionMixin.arg_model() 加载到参数服务器中，
    # 如有需要可在节点参数中通过 LaunchConfiguration 引用它，而不需额外获取

    # 根据 URDF 中的信息解析出机器人基础坐标系和末端执行器坐标系
    # 注意：下面的 "lbr_link_0" 和 "lbr_link_ee" 需与你的 URDF 保持一致
    robot_base_frame = "lbr_link_0"
    robot_effector_frame = "lbr_link_ee"

    # 其它参数直接从 LaunchConfiguration 中获取
    tracking_base_frame = LaunchConfiguration('tracking_base_frame').perform(context)
    tracking_marker_frame = LaunchConfiguration('tracking_marker_frame').perform(context)

    # 修改静态变换发布节点，使其发布的变换连接到 tracking_marker_frame（例如 "marker_link"）
    node_dummy_calib_eih = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dummy_publisher',
        condition=LaunchConfigurationEquals('calibration_type', 'eye_in_hand'),
        arguments=f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') +
                  ['--frame-id', robot_effector_frame, '--child-frame-id', tracking_marker_frame]
    )

    node_dummy_calib_eob = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dummy_publisher',
        condition=LaunchConfigurationEquals('calibration_type', 'eye_on_base'),
        arguments=f'--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') +
                  ['--frame-id', robot_base_frame, '--child-frame-id', tracking_marker_frame]
    )

    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        parameters=[{
            'name': LaunchConfiguration('name').perform(context),
            'calibration_type': LaunchConfiguration('calibration_type').perform(context),
            'tracking_base_frame': tracking_base_frame,
            'tracking_marker_frame': tracking_marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }]
    )

    handeye_rqt_calibrator = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        parameters=[{
            'name': LaunchConfiguration('name').perform(context),
            'calibration_type': LaunchConfiguration('calibration_type').perform(context),
            'tracking_base_frame': tracking_base_frame,
            'tracking_marker_frame': tracking_marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }]
    )

    return [node_dummy_calib_eih, node_dummy_calib_eob, handeye_server, handeye_rqt_calibrator]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # 声明所需的参数
    ld.add_action(DeclareLaunchArgument('name'))
    ld.add_action(DeclareLaunchArgument('calibration_type'))
    ld.add_action(DeclareLaunchArgument('tracking_base_frame', default_value='tr_base', description='跟踪基坐标系'))
    ld.add_action(DeclareLaunchArgument('tracking_marker_frame', default_value='tr_marker', description='跟踪标记坐标系'))

    # 参考 LBR 示例代码，通过 mixin 声明机器人模型参数，从而在 hidden_setup 中可以获取 robot_description
    ld.add_action(LBRDescriptionMixin.arg_model())

    # 使用 OpaqueFunction 延迟构建节点，这样可以在运行时通过 context 获取所有参数及 robot_description
    ld.add_action(OpaqueFunction(function=hidden_setup))

    return ld
