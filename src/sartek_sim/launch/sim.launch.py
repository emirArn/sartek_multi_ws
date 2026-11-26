import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _join_paths(*entries: str) -> str:
    return ":".join([p for p in entries if p])


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("sartek_sim")

    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    world_name = LaunchConfiguration("world_name").perform(context)
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1", "yes")
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    agent_port = LaunchConfiguration("agent_port").perform(context)

    sim_models = os.path.join(pkg_share, "models")
    sim_worlds = os.path.join(pkg_share, "worlds")
    world_sdf = os.path.join(sim_worlds, f"{world_name}.sdf")
    model_sdf = os.path.join(sim_models, vehicle_model, "model.sdf")

    px4_models = os.path.join(px4_dir, "Tools", "simulation", "gz", "models")
    px4_worlds = os.path.join(px4_dir, "Tools", "simulation", "gz", "worlds")
    px4_build = os.path.join(px4_dir, "build", "px4_sitl_default")
    px4_bin = os.path.join(px4_build, "bin", "px4")
    px4_rootfs = os.path.join(px4_build, "rootfs")
    px4_plugins = os.path.join(px4_build, "src", "modules", "simulation", "gz_plugins")
    px4_server_config = os.path.join(
        px4_dir, "src", "modules", "simulation", "gz_bridge", "server.config"
    )

    if not os.path.isfile(world_sdf):
        raise RuntimeError(f"Gazebo dünya dosyası bulunamadı: {world_sdf}")
    if not os.path.isfile(model_sdf):
        raise RuntimeError(f"Uçak model dosyası bulunamadı: {model_sdf}")
    if not os.path.isfile(px4_bin):
        raise RuntimeError(
            f"PX4 SITL binary bulunamadı: {px4_bin}\n"
            f"Önce '{px4_dir}' içinde 'make px4_sitl gz_advanced_plane' çalıştırın."
        )

    gz_resource_path = _join_paths(
        sim_models,
        sim_worlds,
        px4_models,
        px4_worlds,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )
    gz_plugin_path = _join_paths(
        px4_plugins,
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    )

    gz_cmd = ["gz", "sim", "-r"]
    if headless:
        gz_cmd.append("-s")
    gz_cmd.append(world_sdf)

    gazebo_proc = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
            "GZ_IP": "127.0.0.1",
        },
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-file", model_sdf,
            "-name", vehicle_model,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
    )

    px4_proc = ExecuteProcess(
        cmd=[px4_bin, "-d"],
        cwd=px4_rootfs,
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_px4")),
        additional_env={
            "PX4_GZ_STANDALONE": "1",
            "PX4_SYS_AUTOSTART": "4008",
            "PX4_SIM_MODEL": "advanced_plane",
            "PX4_GZ_WORLD": world_name,
            "PX4_GZ_MODEL_NAME": vehicle_model,
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
            "GZ_IP": "127.0.0.1",
        },
    )

    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="front_camera_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_bridge")),
        arguments=["/front_camera"],
        remappings=[("/front_camera", "/camera/image_raw")],
    )

    xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_agent")),
    )

    start_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_node,
            on_exit=[px4_proc, camera_bridge, xrce_agent],
        )
    )

    return [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),
        SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=gz_plugin_path),
        SetEnvironmentVariable(name="GZ_SIM_SERVER_CONFIG_PATH", value=px4_server_config),
        SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1"),
        gazebo_proc,
        TimerAction(period=5.0, actions=[spawn_node]),
        start_after_spawn,
    ]


def generate_launch_description():
    default_px4 = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value=default_px4,
                description="PX4-Autopilot dizini",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="yunuseli",
                description="sartek_sim/worlds altındaki .sdf dünya adı (uzantısız)",
            ),
            DeclareLaunchArgument(
                "vehicle_model",
                default_value="advanced_plane_cam",
                description="sartek_sim/models altındaki uçak model klasörü",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="true ise Gazebo GUI açılmadan (-s) çalışır",
            ),
            DeclareLaunchArgument(
                "start_px4",
                default_value="true",
                description="PX4 SITL'i standalone modda başlat",
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description="Gazebo /front_camera -> ROS 2 /camera/image_raw köprüsünü başlat",
            ),
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description="MicroXRCEAgent udp4 oturumunu başlat",
            ),
            DeclareLaunchArgument(
                "agent_port",
                default_value="8888",
                description="MicroXRCEAgent UDP portu",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.246"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
