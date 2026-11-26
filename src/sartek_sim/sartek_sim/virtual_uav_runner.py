#!/usr/bin/env python3
"""5 sanal hedef İHA'nın Micro XRCE-DDS üzerinden otonom kalkış ve rastgele waypoint uçuş düğümü."""

import math
import random
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    VehicleStatus,
)


class FlightState:
    WAIT_GPS = 0
    TAKEOFF_CMD = 1
    ARM = 2
    CLIMBING = 3
    PRE_OFFBOARD = 4
    SET_OFFBOARD = 5
    WAYPOINT = 6


class VirtualUavRunner(Node):
    """Belirli bir PX4 namespace'indeki sabit kanatlı İHA'yı kaldırıp rastgele waypoint'lerde uçurur."""

    def __init__(self) -> None:
        super().__init__('virtual_uav_runner')

        self.declare_parameter('namespace', 'uav1')
        self.declare_parameter('target_system', 2)
        self.declare_parameter('takeoff_altitude', 50.0)
        self.declare_parameter('waypoint_threshold', 35.0)
        self.declare_parameter('seed', 1)
        self.declare_parameter('send_gcs_heartbeat', False)

        self.ns = self.get_parameter('namespace').value.strip('/')
        self.target_sys = int(self.get_parameter('target_system').value)
        self.takeoff_alt_rel = float(self.get_parameter('takeoff_altitude').value)
        self.wp_threshold = float(self.get_parameter('waypoint_threshold').value)
        seed = int(self.get_parameter('seed').value)
        self._send_gcs_hb = bool(self.get_parameter('send_gcs_heartbeat').value)

        self._rng = random.Random(seed + 100)
        self._base_alt = self.takeoff_alt_rel + (seed % 3) * 10.0
        self._current_wp = self._next_random_waypoint()

        self._status: VehicleStatus | None = None
        self._local_pos: VehicleLocalPosition | None = None
        self._home_alt_amsl: float | None = None
        self._state = FlightState.WAIT_GPS
        self._setpoint_count = 0
        self._tick = 0
        self._hb_seq = 0
        self._gcs_sock = (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self._send_gcs_hb else None
        )
        self._mav_port = 18570 + (self.target_sys - 1)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        prefix = f'/{self.ns}' if self.ns else ''
        self.create_subscription(
            VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v5', self._status_cb, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            f'{prefix}/fmu/out/vehicle_local_position_v1',
            self._local_pos_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleGlobalPosition,
            f'{prefix}/fmu/out/vehicle_global_position',
            self._global_pos_cb,
            px4_qos,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode, f'{prefix}/fmu/in/offboard_control_mode', px4_qos
        )
        self._traj_pub = self.create_publisher(
            TrajectorySetpoint, f'{prefix}/fmu/in/trajectory_setpoint', px4_qos
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand, f'{prefix}/fmu/in/vehicle_command_v1', px4_qos
        )

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info(
            f'[{self.ns}] Hazır (sys_id={self.target_sys}, alt={self._base_alt:.0f}m)'
        )

    def _next_random_waypoint(self) -> tuple[float, float, float]:
        # NED: X=Kuzey, Y=Doğu, Z=Aşağı (negatif=yukarı) — pist çevresi dar alan
        x = self._rng.uniform(-140.0, 140.0)
        y = self._rng.uniform(-120.0, 120.0)
        z = -self._rng.uniform(self._base_alt - 5.0, self._base_alt + 10.0)
        return (x, y, z)

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._status = msg

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._local_pos = msg

    def _global_pos_cb(self, msg: VehicleGlobalPosition) -> None:
        if self._home_alt_amsl is None and math.isfinite(msg.alt) and msg.alt != 0.0:
            self._home_alt_amsl = float(msg.alt)
            self.get_logger().info(f'[{self.ns}] Home AMSL={self._home_alt_amsl:.1f}m')

    def _send_cmd(
        self,
        command: int,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(time.time_ns() // 1000)
        msg.command = int(command)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param3 = float(p3)
        msg.param4 = float(p4)
        msg.param5 = float(p5)
        msg.param6 = float(p6)
        msg.param7 = float(p7)
        msg.target_system = self.target_sys
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._cmd_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float) -> None:
        ts = int(time.time_ns() // 1000)

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = True
        self._offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float('nan')
        self._traj_pub.publish(sp)

    def _send_gcs_heartbeat(self) -> None:
        """QGC açık değilken PX4 'No connection to the GCS' kilidini aşmak için GCS heartbeat gönderir."""
        # MAVLink v1 HEARTBEAT (msgid=0, len=9, sysid=255, compid=190)
        payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 0, 3)
        header = struct.pack('<BBBBBB', 0xFE, 9, self._hb_seq & 0xFF, 255, 190, 0)
        self._hb_seq += 1
        crc = 0xFFFF
        for b in header[1:] + payload + b'\x32':  # HEARTBEAT CRC_EXTRA = 50 (0x32)
            tmp = b ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        pkt = header + payload + struct.pack('<H', crc)
        self._gcs_sock.sendto(pkt, ('127.0.0.1', self._mav_port))

    def _control_loop(self) -> None:
        self._tick += 1
        if self._send_gcs_hb and self._tick % 20 == 1:
            self._send_gcs_heartbeat()

        if self._state == FlightState.WAIT_GPS:
            if self._home_alt_amsl is not None and self._status is not None:
                self._state = FlightState.TAKEOFF_CMD
            return

        if self._state == FlightState.TAKEOFF_CMD:
            if self._tick % 20 == 0:
                target_amsl = self._home_alt_amsl + self._base_alt
                self._send_cmd(
                    VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                    p1=15.0,
                    p4=float('nan'),
                    p5=float('nan'),
                    p6=float('nan'),
                    p7=target_amsl,
                )
                self.get_logger().info(f'[{self.ns}] TAKEOFF komutu gönderildi')
            if (
                self._status is not None
                and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
            ):
                self._state = FlightState.ARM
            return

        if self._state == FlightState.ARM:
            if (
                self._status is not None
                and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            ):
                self.get_logger().info(f'[{self.ns}] Armed -> tırmanıyor')
                self._state = FlightState.CLIMBING
                return
            if self._tick % 20 == 0:
                self._send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0, p2=21196.0
                )
            return

        if self._state == FlightState.CLIMBING:
            if self._local_pos is not None and -self._local_pos.z >= self._base_alt * 0.7:
                self._setpoint_count = 0
                self._state = FlightState.PRE_OFFBOARD
                self.get_logger().info(f'[{self.ns}] Tırmanış tamam -> OFFBOARD hazırlığı')
            return

        wx, wy, wz = self._current_wp
        self._publish_setpoint(wx, wy, wz)

        if self._state == FlightState.PRE_OFFBOARD:
            self._setpoint_count += 1
            if self._setpoint_count >= 40:
                self._state = FlightState.SET_OFFBOARD
            return

        if self._state == FlightState.SET_OFFBOARD:
            if (
                self._status is not None
                and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.get_logger().info(
                    f'[{self.ns}] OFFBOARD aktif -> WP ({wx:.0f}, {wy:.0f}, {-wz:.0f}m)'
                )
                self._state = FlightState.WAYPOINT
                return
            if self._tick % 20 == 0:
                self._send_cmd(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0, p3=0.0
                )
            return

        if self._state == FlightState.WAYPOINT and self._local_pos is not None:
            dx = wx - self._local_pos.x
            dy = wy - self._local_pos.y
            if math.hypot(dx, dy) < self.wp_threshold:
                self._current_wp = self._next_random_waypoint()
                nx, ny, nz = self._current_wp
                self.get_logger().info(
                    f'[{self.ns}] Yeni rastgele WP -> ({nx:.0f}, {ny:.0f}, {-nz:.0f}m)'
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualUavRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._gcs_sock is not None:
            node._gcs_sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
