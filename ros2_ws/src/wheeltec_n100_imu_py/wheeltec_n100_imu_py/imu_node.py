#!/usr/bin/env python3
"""
WheelTEC / FDISYSTEMS N100 IMU (FDILink) ROS 2 Python node

This replaces your current imu_node.py.

It parses FDILink frames:
  [0xFC][type][len][sn][crc8][crc16_hi][crc16_lo][payload...][0xFD]

- crc8 is computed over: 0xFC, type, len, sn
- crc16 is computed over: payload (len bytes)

It publishes:
  - /imu  (sensor_msgs/Imu) from MSG_IMU (type 0x40, payload len 56)

Notes:
- Your hexdump shows type=0x40 and len=0x38(56), which is MSG_IMU.
- If your IMU uses a different message type for IMU data, you can add it similarly.
"""

import struct
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ----------------------- CRC8 (table) -----------------------
CRC8_TABLE = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
]


def crc8_fdilink(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc


# ----------------------- CRC16-CCITT (table) -----------------------
CRC16_TABLE = []


def _init_crc16_table() -> None:
    # Polynomial 0x1021, init=0x0000
    for i in range(256):
        crc = (i << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        CRC16_TABLE.append(crc)


_init_crc16_table()


def crc16_fdilink(payload: bytes) -> int:
    crc = 0x0000
    for b in payload:
        crc = CRC16_TABLE[((crc >> 8) ^ b) & 0xFF] ^ ((crc << 8) & 0xFFFF)
    return crc & 0xFFFF


def read_exact(ser: serial.Serial, n: int) -> bytes:
    """Read exactly n bytes or return b'' if timeout/short read."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return bytes(buf)


def read_fdilink_frame(ser: serial.Serial):
    """
    Returns:
      ("ok", msg_type:int, payload:bytes, sn:int)
      ("bad_crc8", msg_type, length, sn)
      ("bad_crc16", msg_type, length, sn)
      ("bad_end", msg_type, length, sn)
      None (if timed out / incomplete read)
    """
    # Find start 0xFC
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0xFC:
            break

    header_abcd = read_exact(ser, 4)  # type,len,sn,crc8
    if not header_abcd:
        return None

    msg_type = header_abcd[0]
    length = header_abcd[1]
    sn = header_abcd[2]
    crc8_rx = header_abcd[3]

    # CRC8 over [0xFC, type, len, sn]
    if crc8_fdilink(bytes([0xFC, msg_type, length, sn])) != crc8_rx:
        # resync: don't consume more than needed; caller will keep scanning
        return ("bad_crc8", msg_type, length, sn)

    crc16_bytes = read_exact(ser, 2)  # crc16_hi, crc16_lo
    if not crc16_bytes:
        return None
    crc16_rx = (crc16_bytes[0] << 8) | crc16_bytes[1]

    payload = read_exact(ser, length)
    if not payload:
        return None

    end = read_exact(ser, 1)
    if not end:
        return None
    if end[0] != 0xFD:
        return ("bad_end", msg_type, length, sn)

    if crc16_fdilink(payload) != crc16_rx:
        return ("bad_crc16", msg_type, length, sn)

    return ("ok", msg_type, payload, sn)


class N100ImuNode(Node):
    # FDILink message type for calibrated IMU
    MSG_IMU = 0x40
    MSG_IMU_LEN = 56  # observed (0x38)

    def __init__(self):
        super().__init__("imu_node")

        # Use sensor_data QoS by default in many IMU pipelines; CLI can override.
        self.pub = self.create_publisher(Imu, "imu", 10)

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 921600)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_magnetometer", False)  # placeholder, not used
        self.declare_parameter("debug_bad_packets", False)

        port = str(self.get_parameter("serial_port").value)
        baud = int(self.get_parameter("serial_baud").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.debug_bad = bool(self.get_parameter("debug_bad_packets").value)

        try:
            # Ensure raw mode, no flow control; parity/stopbits default to 8N1.
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            # Flush any stale bytes so we start on a clean boundary
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.get_logger().info(f"Opened serial port {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            raise

        # Drive parsing with a timer; keep it fast so buffers don't grow.
        self.timer = self.create_timer(0.002, self.read_and_publish)  # 500 Hz tick

        # Basic stats
        self.ok_count = 0
        self.bad_crc8 = 0
        self.bad_crc16 = 0
        self.bad_end = 0
        self.last_warn_time = self.get_clock().now()

    def _throttled_warn(self, msg: str, period_sec: float = 2.0) -> None:
        now = self.get_clock().now()
        if (now - self.last_warn_time).nanoseconds > int(period_sec * 1e9):
            self.get_logger().warn(msg)
            self.last_warn_time = now

    def read_and_publish(self):
        try:
            # Parse multiple frames per tick if available.
            # We don't rely on in_waiting too much; read_fdilink_frame blocks up to timeout.
            for _ in range(20):
                frame = read_fdilink_frame(self.ser)
                if frame is None:
                    break

                status = frame[0]
                if status != "ok":
                    if status == "bad_crc8":
                        self.bad_crc8 += 1
                    elif status == "bad_crc16":
                        self.bad_crc16 += 1
                    elif status == "bad_end":
                        self.bad_end += 1

                    if self.debug_bad:
                        self._throttled_warn(
                            f"{status}: type=0x{frame[1]:02X} len={frame[2]} sn={frame[3]} "
                            f"(ok={self.ok_count} crc8={self.bad_crc8} crc16={self.bad_crc16} end={self.bad_end})"
                        )
                    continue

                _, msg_type, payload, sn = frame

                # Handle MSG_IMU (0x40)
                if msg_type == self.MSG_IMU:
                    if len(payload) != self.MSG_IMU_LEN:
                        self._throttled_warn(f"MSG_IMU unexpected payload len={len(payload)} (expected {self.MSG_IMU_LEN})")
                        continue

                    # MSG_IMU payload: 12 float32 + int64 timestamp (little endian)
                    # Order per spec: gx gy gz ax ay az mx my mz t_imu press t_press timestamp
                    try:
                        gx, gy, gz, ax, ay, az, mx, my, mz, t_imu, press, t_press, ts = struct.unpack("<12fq", payload)
                    except struct.error as e:
                        self._throttled_warn(f"Unpack error: {e}")
                        continue

                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS time; you can map ts if needed.
                    msg.header.frame_id = self.frame_id

                    # Units:
                    # FDILink docs typically use:
                    #  - gyro: rad/s (or deg/s depending on device config)
                    #  - accel: m/s^2 (or g)
                    # We publish raw values as-is; adjust scaling here if your device outputs different units.
                    msg.angular_velocity.x = float(gx)
                    msg.angular_velocity.y = float(gy)
                    msg.angular_velocity.z = float(gz)

                    msg.linear_acceleration.x = float(ax)
                    msg.linear_acceleration.y = float(ay)
                    msg.linear_acceleration.z = float(az)

                    # Orientation may not be present in MSG_IMU; leave unknown:
                    msg.orientation_covariance[0] = -1.0

                    self.pub.publish(msg)
                    self.ok_count += 1
                else:
                    # Ignore other message types for now
                    continue

        except Exception as e:
            self.get_logger().error(f"Error reading IMU: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = N100ImuNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

