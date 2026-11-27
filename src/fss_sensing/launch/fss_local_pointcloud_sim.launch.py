from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    topic = LaunchConfiguration("local_pc_topic")
    transport = LaunchConfiguration("local_pc_transport")
    default_config_path = PathJoinSubstitution([
        FindPackageShare("fss_sensing"),
        "config",
        "local_pointcloud_sim.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=default_config_path,
            description="Path to the local pointcloud simulator configuration file.",
        ),
        DeclareLaunchArgument("local_pc_topic", default_value="cloud_registered"),
        DeclareLaunchArgument("local_pc_transport", default_value="raw"),
        GroupAction(
            scoped=True,
            actions=[
                SetParameter(
                    name="use_fss_sim_time",
                    value=LaunchConfiguration("use_fss_sim_time"),
                ),
                SetParameter(
                    name="use_sim_time",
                    value=LaunchConfiguration("use_fss_sim_time"),
                ),
                Node(
                    package="fss_sensing",
                    executable="local_pointcloud_sim_node",
                    name="local_pointcloud_sim",
                    output="screen",
                    parameters=[{
                        "config_path": LaunchConfiguration("config_path"),
                    }],
                ),
                Node(
                    package="point_cloud_transport",
                    executable="republish",
                    condition=LaunchConfigurationNotEquals("local_pc_transport", "raw"),
                    parameters=[{
                        "in_transport": "raw",
                        "out_transport": transport,
                    }],
                    remappings=[
                        ("in", topic),
                        (["out/", transport], [topic, "/", transport]),
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
