from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def load_config(context, arguments, vehicles, loaded_configs):
    filename = LaunchConfiguration("config").perform(context)
    vehicles.clear()
    if not filename:
        return
    path = Path(filename)
    with path.open(encoding="utf-8") as stream:
        settings = yaml.safe_load(stream)
    if not isinstance(settings, dict):
        raise ValueError(f"{path}: expected a mapping of launch arguments")
    loaded_configs[path.resolve()] = settings
    if "launch" in settings:
        launch_settings = settings["launch"]
        if not isinstance(launch_settings, dict):
            raise ValueError(f"{path}: launch must be a mapping")
        if "lidar_sim" in settings:
            launch_settings.setdefault("lidar_config", str(path.resolve()))
        settings = launch_settings
    overrides = settings.pop("vehicles", [])
    if not isinstance(overrides, list):
        raise ValueError(f"{path}: vehicles must be a list")
    vehicles.extend(overrides)
    allowed = {argument.name for argument in arguments}
    for name, value in settings.items():
        if name not in allowed:
            raise ValueError(f"{path}: unknown launch argument {name!r}")
        if name in ("lidar_config", "rviz_config"):
            value = (path.parent / value).resolve()
        elif isinstance(value, bool):
            value = str(value).lower()
        context.launch_configurations.setdefault(name, str(value))


def make_drones(context, vehicles, loaded_configs):
    count = int(LaunchConfiguration("num_drones").perform(context))
    drones_per_row = int(LaunchConfiguration("drones_per_row").perform(context))
    startup_batch_size = int(LaunchConfiguration("startup_batch_size").perform(context))
    startup_batch_delay_ms = int(LaunchConfiguration("startup_batch_delay_ms").perform(context))
    if count < 1:
        raise ValueError("num_drones must be at least 1")
    if drones_per_row < 1:
        raise ValueError("drones_per_row must be at least 1")
    if startup_batch_size < 1:
        raise ValueError("startup_batch_size must be at least 1")
    if startup_batch_delay_ms < 0:
        raise ValueError("startup_batch_delay_ms must be non-negative")

    lidar_config = LaunchConfiguration("lidar_config").perform(context)
    lidar = {}
    if lidar_config and LaunchConfiguration("sensing_enable").perform(context) == "true":
        path = Path(lidar_config).resolve()
        config = loaded_configs.get(path)
        if config is None:
            with path.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
        lidar = config.get("lidar_sim", config)
    local_pc_topic = lidar.get("local_pc_topic", "cloud_registered")
    local_pc_transport = lidar.get("local_pc_transport", "raw")

    single_launch = PathJoinSubstitution([
        FindPackageShare("fss_bringup"),
        "launch",
        "sim_px4_drone_lidar_single.launch.py",
    ])
    drone_actions = []
    namespaces = set()
    for offset in range(count):
        index = offset + 1
        vehicle = vehicles[offset] if offset < len(vehicles) else {}
        namespace = vehicle.get("namespace", f"uav{index}").strip("/")
        if not namespace or namespace in namespaces:
            raise ValueError(f"Vehicle {index} needs a unique, non-empty namespace")
        namespaces.add(namespace)
        x, y, z = vehicle.get("position", [offset % drones_per_row, offset // drones_per_row, 1.0])
        drone_actions.append(GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(single_launch),
                    launch_arguments={
                        "namespace": namespace,
                        "mav_sys_id": str(index),
                        "lidar_config": lidar_config,
                        "local_pc_topic": local_pc_topic,
                        "local_pc_transport": local_pc_transport,
                        "use_perfect_drone": LaunchConfiguration("use_perfect_drone"),
                        "use_fss_sim_time": LaunchConfiguration("use_fss_sim_time"),
                        "enable_time_coordinator": "false",
                        "enable_sensing": LaunchConfiguration("sensing_enable"),
                        "enable_rviz": "false",
                        "enable_visualizer": LaunchConfiguration("enable_visualizer"),
                        "vis.visualize_max_freq": LaunchConfiguration("vis.visualize_max_freq"),
                        "vis.enable_history_path": LaunchConfiguration("vis.enable_history_path"),
                        "vis.visualize_path_time": LaunchConfiguration("vis.visualize_path_time"),
                        "vis.color": vehicle.get("color", LaunchConfiguration("vis.color")),
                        "init_x_metre": str(float(x)),
                        "init_y_metre": str(float(y)),
                        "init_z_metre": str(float(z)),
                        "init_yaw_deg": str(float(vehicle.get("yaw_deg", 0.0))),
                    }.items(),
                ),
            ],
        ))
    if startup_batch_delay_ms == 0:
        return drone_actions

    return [
        TimerAction(
            period=(index - 1) // startup_batch_size * startup_batch_delay_ms / 1000.0,
            actions=[drone_action],
        )
        for index, drone_action in enumerate(drone_actions, start=1)
    ]


def generate_launch_description():
    time_coordinator_launch = PathJoinSubstitution([
        FindPackageShare("fss_time"),
        "launch",
        "time_coordinator.launch.py",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fss_bringup"),
        "rviz",
        "sim_px4_drone_lidar_multi.rviz",
    ])

    vehicles = []
    loaded_configs = {}
    arguments = [
        DeclareLaunchArgument("real_time_factor", default_value=LaunchConfiguration("max_real_time_factor", default="1.0")),
        DeclareLaunchArgument("start_coordinator_ui", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("lidar_config", default_value=PathJoinSubstitution([
            FindPackageShare("fss_sensing"), "config", "local_pointcloud_sim.yaml",
        ])),
        DeclareLaunchArgument(
            "num_drones",
            default_value="3",
            description="Number of vehicles to launch.",
        ),
        DeclareLaunchArgument(
            "drones_per_row",
            default_value="10",
            description="Number of vehicles in each formation row; must be at least 1.",
        ),
        DeclareLaunchArgument(
            "startup_batch_size",
            default_value="10",
            description="Number of vehicles to start per batch; must be at least 1.",
        ),
        DeclareLaunchArgument(
            "startup_batch_delay_ms",
            default_value="0",
            description="Delay in milliseconds before starting each vehicle batch.",
        ),
        DeclareLaunchArgument(
            "use_perfect_drone",
            default_value="false",
            choices=["true", "false"],
            description="Use perfect MAVROS-compatible drones instead of PX4 SITL.",
        ),
        DeclareLaunchArgument(
            "use_fss_sim_time",
            default_value="true",
            choices=["true", "false"],
            description="Use the FastSwarmSim coordinated simulation clock.",
        ),
        DeclareLaunchArgument(
            "enable_time_coordinator",
            default_value="true",
            choices=["true", "false"],
            description="Start the FastSwarmSim time coordinator.",
        ),
        DeclareLaunchArgument(
            "sensing_enable",
            default_value="true",
            choices=["true", "false"],
            description="Start the local LiDAR pointcloud simulator for each vehicle.",
        ),
        DeclareLaunchArgument(
            "rviz_enable",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "enable_visualizer",
            default_value="true",
            choices=["true", "false"],
            description="Start vehicle visualizers.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_max_freq",
            default_value="20.0",
            description="Maximum visualization update frequency in Hz.",
        ),
        DeclareLaunchArgument(
            "vis.enable_history_path",
            default_value="true",
            choices=["true", "false"],
            description="Publish vehicle history paths.",
        ),
        DeclareLaunchArgument(
            "vis.visualize_path_time",
            default_value="10.0",
            description="History path duration in seconds.",
        ),
        DeclareLaunchArgument(
            "vis.color",
            default_value="red",
            choices=["red", "green", "blue", "grey"],
            description="Vehicle color used to select the Iris URDF model.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="RViz configuration file used when RViz is enabled.",
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="", description="Optional fleet launch YAML file."),
        OpaqueFunction(function=load_config, args=[arguments, vehicles, loaded_configs]),
        *arguments,
        SetParameter(name="use_fss_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_fss_sim_time")),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(time_coordinator_launch),
                    condition=IfCondition(PythonExpression([
                        "'", LaunchConfiguration("use_fss_sim_time"),
                        "' == 'true' and '", LaunchConfiguration("enable_time_coordinator"),
                        "' == 'true'",
                    ])),
                    launch_arguments={
                        "publish_clock": "true",
                        "start_coordinator_ui": LaunchConfiguration("start_coordinator_ui"),
                        "max_real_time_factor": LaunchConfiguration("real_time_factor"),
                    }.items(),
                ),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz_enable")),
        ),
        OpaqueFunction(function=make_drones, args=[vehicles, loaded_configs]),
    ])
