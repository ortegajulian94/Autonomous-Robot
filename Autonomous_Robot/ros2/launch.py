#Our launch file is responsible for launching our ROS 2 nodes. 
#Make sure all of our nodes have the parameter ‘use_sim_time’ set to True. 
#With this set, all of our nodes will be subscribed to the /clock topic. 



from inspect import Arguments, Parameter
import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, ThisLaunchFileDir
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robotech_python_package').find('robotech_python_package')
    default_model_path = os.path.join(pkg_share, 'src/description/robotech.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},{"publish_frequency": 50.0},{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    odom_node = Node(
    	package='robotech_python_package',
    	executable='odom_node',
    	name='odom_node',
    	output='screen',
    	parameters=[{'update_frequency': 115200}]
    )

    

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        odom_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node

    ])
