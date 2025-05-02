from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node



def generate_launch_description():
    arg_name = DeclareLaunchArgument('name',default_value ='tr_base')

    system_rqt_calibrator = Node(package='camera', executable='rqt_system.py',
                                  name='system_rqt',
                                  parameters=[
                                    {'name': LaunchConfiguration('name')}
                                  ]
                                  )

    return LaunchDescription([
        arg_name,
        system_rqt_calibrator
    ])
