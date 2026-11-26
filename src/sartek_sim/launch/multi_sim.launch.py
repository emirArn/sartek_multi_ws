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
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _join_paths(*entries: str) -> str:
    return ":".join([p for p in entries if p])


def _is_qgc_running() -> bool:
    """Sistemde çalışan bir QGroundControl süreci olup olmadığını kontrol eder."""
    my_pid = str(os.getpid())
    try:
        for entry in os.listdir("/proc"):
            if not entry.isdigit() or entry == my_pid:
                continue
            try:
                with open(f"/proc/{entry}/comm", "r", encoding="utf-8", errors="ignore") as f:
                    if "qgroundcontrol" in f.read().lower():
                        return True
                with open(f"/proc/{entry}/cmdline", "rb") as f:
                    if b"qgroundcontrol" in f.read().lower():
                        return True
            except OSError:
                continue
    except OSError:
        return False
    return False


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("sartek_sim")

    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    world_name = LaunchConfiguration("world_name").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1", "yes")
    agent_port = LaunchConfiguration("agent_port").perform(context)
    auto_fly = LaunchConfiguration("auto_fly").perform(context).lower() in ("true", "1", "yes")
    require_qgc = LaunchConfiguration("require_qgc").perform(context).lower() in ("true", "1", "yes")

    if require_qgc and not _is_qgc_running():
        raise RuntimeError(
            "QGroundControl açık değil! 6 uçağın tamamını QGC'de görebilmek için önce "
            "QGroundControl'ü başlatın (~/.local/bin/qgroundcontrol), ardından bu launch'ı "
            "tekrar çalıştırın. (QGC olmadan çalıştırmak için: require_qgc:=false)"
        )

    sim_models = os.path.join(pkg_share, "models")
    sim_worlds = os.path.join(pkg_share, "worlds")
    world_sdf = os.path.join(sim_worlds, f"{world_name}.sdf")
    cam_model_sdf = os.path.join(sim_models, "advanced_plane_cam", "model.sdf")

    px4_models = os.path.join(px4_dir, "Tools", "simulation", "gz", "models")
    px4_worlds = os.path.join(px4_dir, "Tools", "simulation", "gz", "worlds")
    plane_model_sdf = os.path.join(px4_models, "advanced_plane", "model.sdf")

    px4_build = os.path.join(px4_dir, "build", "px4_sitl_default")
    px4_bin = os.path.join(px4_build, "bin", "px4")
    px4_rootfs = os.path.join(px4_build, "rootfs")
    px4_plugins = os.path.join(px4_build, "src", "modules", "simulation", "gz_plugins")
    px4_server_config = os.path.join(
        px4_dir, "src", "modules", "simulation", "gz_bridge", "server.config"
    )

    for path in (world_sdf, cam_model_sdf, plane_model_sdf, px4_bin):
        if not os.path.isfile(path):
            raise RuntimeError(f"Gerekli dosya bulunamadı: {path}")

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

    common_gz_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
        "GZ_SIM_SERVER_CONFIG_PATH": px4_server_config,
        "GZ_IP": "127.0.0.1",
    }

    gz_cmd = ["gz", "sim", "-r"]
    if headless:
        gz_cmd.append("-s")
    gz_cmd.append(world_sdf)

    gazebo_proc = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env=common_gz_env,
    )

    xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
        output="screen",
    )

    # ── Kendi uçağımız (6. uçak: advanced_plane_cam, instance 0, kök namespace) ──
    cam_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-file", cam_model_sdf,
            "-name", "advanced_plane_cam",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.246",
            "-Y", "0.0",
        ],
    )
    cam_px4 = ExecuteProcess(
        cmd=[px4_bin, "-d", "-i", "0"],
        cwd=px4_rootfs,
        output="screen",
        additional_env={
            **common_gz_env,
            "PX4_GZ_STANDALONE": "1",
            "PX4_SYS_AUTOSTART": "4008",
            "PX4_SIM_MODEL": "advanced_plane",
            "PX4_GZ_WORLD": world_name,
            "PX4_GZ_MODEL_NAME": "advanced_plane_cam",
            "PX4_HOME_LAT": "40.233243",
            "PX4_HOME_LON": "29.005583",
            "PX4_HOME_ALT": "101.0",
        },
    )
    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="front_camera_bridge",
        output="screen",
        arguments=["/front_camera"],
        remappings=[("/front_camera", "/camera/image_raw")],
    )
    cam_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=cam_spawn,
            on_exit=[cam_px4, camera_bridge],
        )
    )

    actions = [
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_resource_path),
        SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=gz_plugin_path),
        SetEnvironmentVariable(name="GZ_SIM_SERVER_CONFIG_PATH", value=px4_server_config),
        SetEnvironmentVariable(name="GZ_IP", value="127.0.0.1"),
        gazebo_proc,
        TimerAction(period=2.0, actions=[xrce_agent]),
        TimerAction(period=5.0, actions=[cam_spawn]),
        cam_after_spawn,
    ]

    # ── 5 sanal rakip/otonom uçak (gz_advanced_plane_1..5, instance 1..5) ──
    virtual_uavs = [
        ("gz_advanced_plane_1", "uav1", 1, "0.0", "12.0", 8.0),
        ("gz_advanced_plane_2", "uav2", 2, "0.0", "-12.0", 11.0),
        ("gz_advanced_plane_3", "uav3", 3, "-18.0", "12.0", 14.0),
        ("gz_advanced_plane_4", "uav4", 4, "-18.0", "-12.0", 17.0),
        ("gz_advanced_plane_5", "uav5", 5, "-36.0", "0.0", 20.0),
    ]

    for model_name, dds_ns, instance, sx, sy, delay in virtual_uavs:
        spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-world", world_name,
                "-file", plane_model_sdf,
                "-name", model_name,
                "-x", sx,
                "-y", sy,
                "-z", "0.246",
                "-Y", "0.0",
            ],
        )
        px4_proc = ExecuteProcess(
            cmd=[px4_bin, "-d", "-i", str(instance)],
            cwd=px4_rootfs,
            output="screen",
            additional_env={
                **common_gz_env,
                "PX4_GZ_STANDALONE": "1",
                "PX4_SYS_AUTOSTART": "4008",
                "PX4_SIM_MODEL": "advanced_plane",
                "PX4_GZ_WORLD": world_name,
                "PX4_GZ_MODEL_NAME": model_name,
                "PX4_UXRCE_DDS_NS": dds_ns,
                "PX4_HOME_LAT": "40.233243",
                "PX4_HOME_LON": "29.005583",
                "PX4_HOME_ALT": "101.0",
            },
        )
        after_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[px4_proc],
            )
        )
        actions.append(TimerAction(period=delay, actions=[spawn_node]))
        actions.append(after_spawn)

        if auto_fly:
            runner = Node(
                package="sartek_sim",
                executable="virtual_uav_runner",
                name=f"virtual_uav_runner_{dds_ns}",
                output="screen",
                parameters=[
                    {
                        "namespace": dds_ns,
                        "target_system": instance + 1,
                        "takeoff_altitude": 50.0,
                        "waypoint_threshold": 35.0,
                        "seed": instance,
                        "send_gcs_heartbeat": not require_qgc,
                    }
                ],
            )
            actions.append(TimerAction(period=delay + 15.0, actions=[runner]))

    return actions


def generate_launch_description():
    default_px4 = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_dir", default_value=default_px4),
            DeclareLaunchArgument("world_name", default_value="yarisma"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("agent_port", default_value="8888"),
            DeclareLaunchArgument("auto_fly", default_value="true"),
            DeclareLaunchArgument("require_qgc", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
