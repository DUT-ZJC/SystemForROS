cmake_minimum_required(VERSION 3.5)
project(execute_movement_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
if("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "iron")
    find_package(service_msgs REQUIRED) # should be automatically added, it doesn't because we have subdirs in srv :\
    set(SERVICE_MSGS_DEPS "service_msgs")
endif()
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        
        srv/AutoMove_Start.srv
        srv/Keep_Move.srv
        srv/Move_To_Strat.srv
        srv/Stop_And_Back.srv
        srv/Stop_Move.srv
        DEPENDENCIES std_msgs geometry_msgs ${SERVICE_MSGS_DEPS}
)

ament_package()