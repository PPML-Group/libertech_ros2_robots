from ament_index_python.packages import get_package_share_path
import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
# from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, ExecuteProcess

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_hand_driver(context, *args, **kwargs):
    # urdf_path = get_package_share_path('libertech_hand_description')
    urdf_file_name = 'hand_' + \
        launch.substitutions.LaunchConfiguration('hand_type').perform(context) + ".xacro.urdf"
    
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "urdf", urdf_file_name]
    )
    
    
    return [
        Node(
            package='libertech_hand_driver',
            executable='hand_demo.py',
            name='hand_demo',
            parameters=[{'urdf_file_path': urdf_file},
                        {'actions_file_path': LaunchConfiguration("action_file")},
                        {'ip': LaunchConfiguration("hand_ip")}],
            output='screen',
        )
    ]


    # hand_controller_node = Node(
    #         package='libertech_hand_driver',
    #         executable='hand_demo.py',
    #         name='hand_demo',
    #         parameters=[{'urdf_file_path': urdf_file},
    #                     {'actions_file_path': action_file},
    #                     {'ip': hand_ip}],
    #         output='screen',
    #     )


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_type",
            default_value="right",
            description="Type of hand",
            choices=["right", "left"],
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand_ip",
            default_value='10.10.20.61',
            description="IP of Hand",
        )
    )
    # prefix = LaunchConfiguration("prefix")
    # hand_type = LaunchConfiguration("hand_type")
    # hand_ip = LaunchConfiguration("hand_ip")
   
    # urdf_file = launch.actions.OpaqueFunction(function=launch_hand_driver),
    
    
    # print(PythonExpression(['str(', hand_type, ')']))
    # hand_type_str = PythonExpression(['str(', hand_type, ')'])
    
    # print(str(hand_type.perform(launch.LaunchContext())))
    # hand_urdf = hand_type
    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "hand_urdf",
    #         default_value=f'{urdf_path}/urdf/hand_{hand_type_str}.xacro.urdf',
    #         description="Another argument using the value of hand_type",
    #     ),
    # )
    # hand_urdf = LaunchConfiguration("hand_urdf")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_description"), "urdf", "hand.xacro"]
    )
    # hand_urdf = PathJoinSubstitution(
    #     [FindPackageShare("libertech_hand_description"), "urdf", "hand_right.xacro.urdf"]
    # )
    # # rviz_file = PathJoinSubstitution(
    # #     [FindPackageShare("libertech_hand_description"), "rviz", "urdf.rviz"]
    # # )
    action_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_driver"), "config", "actions.yaml"]
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "action_file",
            default_value=action_file,
            description="action_file",
        )
    )
    # # urdf_file = urdf_path / 
    # # xacro_to_urdf = ExecuteProcess(
    # #         # cmd=['xacro', xacro_file, '-o', f"{urdf_path}/urdf/hand2.xacro.urdf", 'prefix:=',prefix,"type:=",hand_type],
    # #         cmd=['xacro', xacro_file],
    # #         output='screen',
    # #         log_cmd=True,
    # #         shell='True',
    # #         name='xacro_to_urdf',
    # #     ),
    
    # # cmd_arg = ["prefix:=",
    # #     prefix,
    # #     "type:=",
    # #     hand_type,
    # # ]
    # # cmd_arg = [arg.strip() for arg in cmd_arg]
    # # xacro_to_urdf = ExecuteProcess(
    # #     cmd=[
    # #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    # #         xacro_file,
    # #         "-o",
    # #         f"{urdf_path}/urdf/hand.xacro.urdf",
    # #         "prefix:=",
    # #         prefix.perform(str),
    # #         "type:=",
    # #         hand_type.perform(str),
    # #     ],

    # #     output='screen',
    # #     log_cmd=True,
    # #     shell=True,
    # #     name='xacro_to_urdf',
    # # )   
    # # xacro_to_urdf = Node(
    # #         package='xacro',
    # #         executable='xacro',
    # #         name='xacro_to_urdf',
    # #         output='screen',
    # #         arguments=['your_robot_description.urdf.xacro'],
    # #     ),
    
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         xacro_file,
    #         " ",
    #         "prefix:=",
    #         prefix,
    #         " ",
    #         "type:=",
    #         hand_type,
    #         " ",
    #     ]
    # )
    # urdf_file = urdf_path / 'urdf/hand_urdf_tmp.urdf'
    # with open(urdf_file, 'w') as file:
    #     file.write(robot_description_content)
    
    # # xacro_to_urdf = Command(
    # #     [
    # #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    # #         " ",
    # #         xacro_file,
    # #         " ",
    # #         "-o",
    # #         " ",
    # #         f"{urdf_path}/urdf/hand.xacro.urdf",
    # #         " ",
    # #         "prefix:=",
    # #         prefix,
    # #         " ",
    # #         "type:=",
    # #         hand_type,
    # #         " ",
    # #     ]
    # # )
    # # print(robot_urdf)

    # # driver_path = get_package_share_path('libertech_hand_driver')
    # # declared_arguments.append(
    # #     DeclareLaunchArgument(
    # #         "action_file",
    # #         default_value=f'{driver_path}/config/actions.yaml',
    # #         description="Absolute path to action yaml file",
    # #     )
    # # )
    
    
    

    nodes_to_start = [
        # xacro_to_urdf,
        # hand_controller_node,
        launch.actions.OpaqueFunction(function=launch_hand_driver),
    ]
    # return LaunchDescription(nodes_to_start)
    return LaunchDescription(declared_arguments + nodes_to_start)
