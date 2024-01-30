from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # name = 'right'
    name = 'left'
    urdf_package_path = get_package_share_path('libertech_hand_description')
    default_model_path = urdf_package_path / f'urdf/{name}_hand.urdf'
    default_rviz_config_path = urdf_package_path / f'rviz/urdf_{name}.rviz'
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    
    controller_path = get_package_share_path('libertech_hand_driver')
    actions_path = controller_path / f'config/actions.yaml'
    actions_arg = DeclareLaunchArgument(name='actions_path', default_value=str(actions_path),
                                     description='Absolute path to actions yaml file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    hand_driver_node = Node(
        package='libertech_hand_driver',
        executable='hand_ros_driver.py',
        name='hand_ros_driver',
        parameters=[{'urdf_file_path': LaunchConfiguration('model')},
                    {'actions_file_path': LaunchConfiguration('actions_path')}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    rqt_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        actions_arg,
        hand_driver_node,
        TimerAction(period=5.0, actions=[robot_state_publisher_node]),
        rviz_node,
        rqt_node,
    ])
