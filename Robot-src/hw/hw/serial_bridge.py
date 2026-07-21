"""
Serial Bridge for Delta Robot - ENHANCED VERSION
Adds missing command interfaces for manual operation
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import threading
import time
import struct
import re

from std_msgs.msg import Bool, UInt8, UInt16, UInt8MultiArray, String, Empty
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from msgs.msg import RobotStatus, TrajectoryCommand


START_BYTE = 0xAA
END_BYTE = 0x55

# Commands (ESP32 → from protocol.h)
CMD_HOME = 0x01
CMD_EMERGENCY_STOP = 0x02
CMD_ENABLE_MOTORS = 0x03
CMD_DISABLE_MOTORS = 0x04
CMD_SET_VACUUM = 0x05
CMD_SET_SERVO = 0x06
CMD_TRAJECTORY = 0x10
CMD_GET_STATUS = 0x20
CMD_GET_ENCODERS = 0x21
CMD_INIT = 0x22

# Responses
RSP_STATUS = 0x90
RSP_ACK = 0x80
RSP_TRAJECTORY_COMPLETE = 0x92
RSP_ENCODER_DATA = 0x93


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameters('', [
            ('port', '/dev/ttyACM0'),
            ('baudrate', 115200),
            ('auto_home_on_connect', False),  # Optional: auto-home at startup
            ('auto_enable_on_connect', False), # Optional: auto-enable at startup
        ])

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.ser = None
        self.lock = threading.Lock()
        self.connected = False
        self.running = True

        # State tracking
        self.motors_enabled = False
        self.is_homed = False
        self.last_status = None
        self.synchronized = False

        # Publishers
        self.status_pub = self.create_publisher(RobotStatus, '/robot/status', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.rx_raw_pub = self.create_publisher(UInt8MultiArray, '/serial/rx_raw', 10)
        self.tx_raw_pub = self.create_publisher(UInt8MultiArray, '/serial/tx_raw', 10)
        self.rx_text_pub = self.create_publisher(String, '/serial/rx_text', 10)
        self.traj_complete_pub = self.create_publisher(UInt16, '/robot/trajectory_complete', 10)

        # Subscribers 
        self.create_subscription(TrajectoryCommand, '/robot/commands', self.on_trajectory, 10)
        self.create_subscription(Bool, '/gripper/command', self.on_gripper, 10)
        self.create_subscription(UInt8, '/servo/command', self.on_servo, 10)
        self.create_subscription(Empty, '/robot/init', self.on_init, 10)

        # Direct command subscribers for manual control
        self.create_subscription(Empty, '/robot/home', self.on_home, 10)
        self.create_subscription(Bool, '/robot/enable_motors', self.on_enable_motors, 10)
        self.create_subscription(Empty, '/robot/emergency_stop', self.on_emergency_stop, 10)
        self.create_subscription(Empty, '/robot/get_status', self.on_get_status, 10)
        self.create_subscription(Empty, '/robot/get_encoders', self.on_get_encoders, 10)

        # Services (blocking calls with feedback)
        self.srv_home = self.create_service(Trigger, '/robot/home_srv', self.srv_home_callback)
        self.srv_enable = self.create_service(Trigger, '/robot/enable_srv', self.srv_enable_callback)

        self.rx_buf = bytearray()
        self.text_buffer = ""

        self.connect()

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f'Serial bridge on {self.port}@{self.baudrate}')
        
        # Optional auto-setup
        if self.get_parameter('auto_home_on_connect').value:
            time.sleep(0.5)
            self.send_home()
        if self.get_parameter('auto_enable_on_connect').value:
            time.sleep(0.5)
            self.send_enable(True)

    def send_init(self):
        self.get_logger().info("Sending INIT command")
        pkt = self.encode_packet(CMD_INIT, b'')
        return self.send(pkt)

    def send_home(self):
        """Send HOME command to ESP32"""
        self.get_logger().info("Sending HOME command")
        pkt = self.encode_packet(CMD_HOME, b'')
        return self.send(pkt)

    def send_enable(self, enable=True):
        """Send ENABLE/DISABLE command"""
        cmd = CMD_ENABLE_MOTORS if enable else CMD_DISABLE_MOTORS
        state = "ENABLE" if enable else "DISABLE"
        self.get_logger().info(f"Sending {state}_MOTORS")
        pkt = self.encode_packet(cmd, b'')
        return self.send(pkt)

    def send_emergency_stop(self):
        """Send emergency stop"""
        self.get_logger().warn("Sending EMERGENCY STOP!")
        pkt = self.encode_packet(CMD_EMERGENCY_STOP, b'')
        return self.send(pkt)

    def send_get_status(self):
        """Request immediate status"""
        pkt = self.encode_packet(CMD_GET_STATUS, b'')
        return self.send(pkt)

    def send_get_encoders(self):
        """Request encoder values"""
        pkt = self.encode_packet(CMD_GET_ENCODERS, b'')
        return self.send(pkt)

    def on_init(self, msg):
        """Trigger initialization sequence"""
        self.send_init()

    def on_home(self, msg):
        """Trigger message to home robot"""
        self.send_home()

    def on_enable_motors(self, msg):
        """Bool message: true=enable, false=disable"""
        self.send_enable(msg.data)

    def on_emergency_stop(self, msg):
        """Trigger emergency stop"""
        self.send_emergency_stop()

    def on_get_status(self, msg):
        """Trigger status request"""
        self.send_get_status()

    def on_get_encoders(self, msg):
        """Trigger encoder request"""
        self.send_get_encoders()

    def srv_home_callback(self, request, response):
        """Service version of home with confirmation"""
        self.get_logger().info("Service: HOME requested")
        success = self.send_home()
        
        # Wait for completion (simple polling)
        timeout = 10.0  # seconds
        start = time.time()
        while time.time() - start < timeout:
            if self.last_status and self.last_status.state == 0:  # IDLE
                response.success = True
                response.message = "Homing complete"
                return response
            time.sleep(0.1)
        
        response.success = False
        response.message = "Homing timeout or failed"
        return response

    def srv_enable_callback(self, request, response):
        """Service to enable/disable motors"""
        self.send_enable(True)
        response.success = True
        response.message = "Enable command sent"
        return response

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.connected = True
            self.get_logger().info("Connected to ESP32")
        except Exception as e:
            self.get_logger().error(f"Connection failed: {e}")
            self.connected = False

    def crc8(self, data: bytes):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def encode_packet(self, cmd: int, payload: bytes = b''):
        pkt = bytearray([START_BYTE, len(payload) + 1, cmd])
        pkt.extend(payload)
        pkt.append(self.crc8(pkt[1:]))
        pkt.append(END_BYTE)
        return bytes(pkt)

    def send(self, data: bytes):
        if not self.connected:
            return False
        try:
            msg = UInt8MultiArray()
            msg.data = list(data)
            with self.lock:
                self.ser.write(data)
            self.tx_raw_pub.publish(msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")
            self.connected = False
            return False

    def on_gripper(self, msg: Bool):
        payload = struct.pack('B', 1 if msg.data else 0)
        packet = self.encode_packet(CMD_SET_VACUUM, payload)
        self.send(packet)

    def on_servo(self, msg: UInt8):
        payload = struct.pack('B', msg.data)
        packet = self.encode_packet(CMD_SET_SERVO, payload)
        self.send(packet)
        
    def on_trajectory(self, msg):
        # ============================================================
        # Metadata
        # ============================================================

        trajectory_id = int(msg.trajectory_id)
        num_points = len(msg.points)

        # ============================================================
        # Header
        # ============================================================

        payload = struct.pack(
            '<HH',
            int(msg.trajectory_id),
            int(num_points)
        )

        # ============================================================
        # Points
        # ============================================================

        for pt in msg.points:

            payload += struct.pack(
                '<fffIBBH',
                float(pt.joint_angles[0]),
                float(pt.joint_angles[1]),
                float(pt.joint_angles[2]),
                int(pt.time_ms),
                int(pt.vacuum),
                int(pt.servo_angle),
                int(pt.reserved)
            )

        # ============================================================
        # Send
        # ============================================================

        packet = self.encode_packet(
            CMD_TRAJECTORY,
            payload
        )

        self.send(packet)



    def read_loop(self):
        while self.running:
            if not self.connected:
                time.sleep(2.0)
                self.connect()
                continue
            try:
                with self.lock:
                    if self.ser.in_waiting:
                        data = self.ser.read(self.ser.in_waiting)
                        if data:
                            raw_msg = UInt8MultiArray()
                            raw_msg.data = list(data)
                            self.rx_raw_pub.publish(raw_msg)
                            self.process_text_stream(data)
                            self.rx_buf.extend(data)
                self.process_buffer()
            except Exception as e:
                self.get_logger().error(f"Read error: {e}")
                self.connected = False
            time.sleep(0.001)

    def process_text_stream(self, data: bytes):
        try:
            text = data.decode('utf-8', errors='ignore')
        except:
            return
        self.text_buffer += text
        while "\n" in self.text_buffer:
            line, self.text_buffer = self.text_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            ansi_escape = re.compile(r'\x1B\[[0-?]*[ -/]*[@-~]')
            line = ansi_escape.sub('', line)
            msg = String()
            msg.data = line
            self.rx_text_pub.publish(msg)
    
    def process_buffer(self):
        while len(self.rx_buf) >= 5:
            try:
                start = self.rx_buf.index(START_BYTE)
            except ValueError:
                self.rx_buf.clear()
                return
            self.rx_buf = self.rx_buf[start:]
            if len(self.rx_buf) < 2:
                return
            length = self.rx_buf[1]
            pkt_size = length + 4
            if len(self.rx_buf) < pkt_size:
                return
            if self.rx_buf[pkt_size - 1] != END_BYTE:
                self.get_logger().warn("Packet end byte mismatch")
                self.rx_buf = self.rx_buf[1:]
                continue
            pkt = bytes(self.rx_buf[:pkt_size])
            self.rx_buf = self.rx_buf[pkt_size:]
            if self.crc8(pkt[1:-2]) == pkt[-2]:
                self.handle_packet(pkt[2], pkt[3:-2])
                self.get_logger().debug(f"Received packet: cmd=0x{pkt[2]:02X}, payload={pkt[3:-2].hex()}")
            else:
                self.get_logger().warn("CRC mismatch on received packet")

    def handle_packet(self, msg_type, payload):
        if msg_type == RSP_STATUS:
            self.handle_status(payload)
        elif msg_type == RSP_TRAJECTORY_COMPLETE and len(payload) >= 2:
            traj_id = struct.unpack('<H', payload[:2])[0]
            self.get_logger().info(f"Trajectory {traj_id} complete")

            msg = UInt16()
            msg.data = traj_id
            self.traj_complete_pub.publish(msg)

    def handle_status(self, payload):
        n = len(payload)
        if n < 36:
            self.get_logger().warn(f"Status packet too short: {n}")
            return
        try:
            state = payload[0]
            error_code = payload[1]
            offset = 3
            angles = struct.unpack('<fff', payload[offset:offset+12])
            offset += 12
            steps = struct.unpack('<iii', payload[offset:offset+12])
            offset += 12
            encoders = struct.unpack('<HHH', payload[offset:offset+6])
            offset += 6
            
            flags = payload[offset]
            offset += 1
            
            if n >= 37:
                # New format: uint16 little-endian
                traj_id = struct.unpack('<H', payload[offset:offset+2])[0]
                offset += 2
            else:
                # Old format: uint8 (legacy fallback)
                traj_id = payload[offset]
                offset += 1
                
            progress = payload[offset] / 100.0

            status = RobotStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.state = state
            status.error_code = error_code
            status.current_angles = list(angles)
            status.encoder_values = [float(e) for e in encoders]
            status.motors_enabled = bool(flags & 0x01)
            status.vacuum_on = bool(flags & 0x02)
            status.synchronized = bool(flags & 0x10)
            status.trajectory_id = traj_id
            status.trajectory_progress = progress
            self.last_status = status
            self.status_pub.publish(status)

            js = JointState()
            js.header.stamp = status.header.stamp
            js.name = ['joint1','joint2','joint3']
            js.position = [a * 0.0174533 for a in angles]
            js.effort = [float(s) for s in steps]
            self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().error(f"Status parse error: {e}")

    def destroy_node(self):
        self.running = False
        self.read_thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()