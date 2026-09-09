#!/usr/bin/env python3
import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import glob
import time

def find_esp_port():
    # Prefer ACM, then USB
    candidates = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise FileNotFoundError("No ESP serial device found. Check /dev/ttyACM* or /dev/ttyUSB*")
    return candidates[0]

class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge_usb')

        # Parameters
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        port_param = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        port = port_param.strip() if port_param.strip() else find_esp_port()

        # Serial: non-blocking via timeout=0
        self.ser = serial.Serial(port, baudrate=baud, timeout=0)
        time.sleep(0.2)  # let it settle

        self.mode = 'MANUAL'  # default
        self.last_twist = Twist()

        self.sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)

        # Timer drives everything in a single ROS thread
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

        self.get_logger().info(f"Opened serial: {port} @ {baud}")
        self.print_help()

        # Send initial mode to ESP32
        self.send_line("MANUAL")

    def print_help(self):
        print("\n=== ESP32 Bridge Controls (type then press Enter) ===")
        print("  auto    -> switch to AUTO mode (forwards /cmd_vel to ESP32)")
        print("  manual  -> switch to MANUAL mode (stops forwarding /cmd_vel)")
        print("  stop    -> send a stop cmd_vel once (AUTO only forwards live /cmd_vel)")
        print("  q       -> quit")
        print("====================================================\n")

    def on_cmd_vel(self, msg: Twist):
        self.last_twist = msg

    def send_line(self, s: str):
        try:
            self.ser.write((s.strip() + "\n").encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def format_twist(self, t: Twist) -> str:
        # Simple text protocol: "CMD_VEL lin_x ang_z"
        # Keep it consistent with your ESP32 parser
        return f"CMD_VEL {t.linear.x:.4f} {t.angular.z:.4f}"

    def handle_user_input(self):
        # Non-blocking stdin read
        if select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline()
            if not line:
                return
            cmd = line.strip().lower()

            if cmd in ("q", "quit", "exit"):
                print("Quitting...")
                rclpy.shutdown()
                return

            if cmd == "auto":
                self.mode = "AUTO"
                self.send_line("AUTO")
                print("[Pi5] Mode set to AUTO. Forwarding /cmd_vel to ESP32.")
                return

            if cmd == "manual":
                self.mode = "MANUAL"
                self.send_line("MANUAL")
                print("[Pi5] Mode set to MANUAL. Not forwarding /cmd_vel.")
                return

            if cmd == "stop":
                # Send a one-shot stop
                stop_twist = Twist()
                self.send_line(self.format_twist(stop_twist))
                print("[Pi5] Sent STOP cmd_vel (0,0).")
                return

            if cmd == "help":
                self.print_help()
                return

            print(f"[Pi5] Unknown command: {cmd} (type 'help')")

    def read_serial_lines(self):
        # Read any available serial bytes, parse by newline
        try:
            data = self.ser.read(4096)
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        if not data:
            return

        # Accumulate and split
        if not hasattr(self, "_rx_buf"):
            self._rx_buf = ""

        self._rx_buf += data.decode('utf-8', errors='ignore')
        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            # Print ESP messages directly to terminal
            print(f"[ESP32] {line}")

    def loop(self):
        # 1) handle user commands from terminal
        self.handle_user_input()

        # 2) forward cmd_vel only in AUTO
        if self.mode == "AUTO":
            self.send_line(self.format_twist(self.last_twist))

        # 3) read and print ESP32 feedback (ACK + ECHO)
        self.read_serial_lines()

def main():
    rclpy.init()
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
