from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description(): 
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="warrior_description",
            description="Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.",
    )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="warrior.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="robot_controller_config.yaml",
            description="YAML file with the controllers configuration.",
        )
    )  
     
    controllers_file = LaunchConfiguration("controllers_file")     
    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_hardware = LaunchConfiguration("use_hardware")    
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "use_hardware:=",
            use_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            controllers_file,
        ]
    )    
    
    control_node = Node(
        package='warrior_control_manager',
        executable='warrior_control_manager',
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    dbus_node = Node(
        package='warrior_dbus',
        executable='dbus_node',
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    nodes = [
        control_node,
#        dbus_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)