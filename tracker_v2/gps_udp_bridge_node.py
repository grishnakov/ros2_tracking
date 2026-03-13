"""
Receive GPS text lines over UDP and publish ROS NavSatFix messages.

Supported line formats:
- Point One runner output containing `LLA=<lat>, <lon>, <alt>`
- NMEA RMC sentences like `$GPRMC,...*hh` / `$GNRMC,...*hh`
"""

import math
import re
import socket
from functools import reduce
import operator

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

NODE_NAME = 'gps_udp_bridge_node'


class GPSUdpBridgeNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('listen_host', '0.0.0.0'),
                ('listen_port', 10110),
                ('fix_topic', '/fix'),
                ('frame_id', 'gps'),
                ('publish_invalid_fixes', False),
                ('verbose_udp_logging', False),
            ],
        )

        self.listen_host = str(self.get_parameter('listen_host').value)
        self.listen_port = int(self.get_parameter('listen_port').value)
        self.fix_topic = str(self.get_parameter('fix_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_invalid_fixes = bool(self.get_parameter('publish_invalid_fixes').value)
        self.verbose_udp_logging = bool(self.get_parameter('verbose_udp_logging').value)

        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self._lla_pattern = re.compile(
            r'LLA=\s*(?P<lat>[^,]+),\s*(?P<lon>[^,]+),\s*(?P<alt>[^\]]+)'
        )
        self._last_fix_log_time = 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_host, self.listen_port))
        self.sock.setblocking(False)

        self.create_timer(0.02, self._poll_socket)

        self.get_logger().info(
            f'{NODE_NAME} started'
            f'\n  listening on udp://{self.listen_host}:{self.listen_port}'
            f'\n  fix_topic: {self.fix_topic}'
            f'\n  frame_id: {self.frame_id}'
        )

    def _poll_socket(self) -> None:
        while True:
            try:
                payload, _addr = self.sock.recvfrom(8192)
            except BlockingIOError:
                break
            except Exception as exc:
                self.get_logger().error(f'UDP receive failed: {exc}')
                break

            text = payload.decode('utf-8', errors='ignore')
            for line in text.splitlines():
                stripped = line.strip()
                if not stripped:
                    continue
                if self.verbose_udp_logging:
                    self.get_logger().info(f'[udp] {stripped}')
                self._handle_line(stripped)

    def _handle_line(self, line: str) -> None:
        fix = self._parse_lla_line(line)
        if fix is None:
            fix = self._parse_nmea_line(line)
        if fix is None:
            return

        lat, lon, alt, is_valid, source = fix
        if not is_valid and not self.publish_invalid_fixes:
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.status.status = (
            NavSatStatus.STATUS_FIX if is_valid else NavSatStatus.STATUS_NO_FIX
        )
        msg.status.service = NavSatStatus.SERVICE_GPS
        self.fix_pub.publish(msg)

        now = self.get_clock().now().nanoseconds / 1e9
        if is_valid and (self.verbose_udp_logging or (now - self._last_fix_log_time) >= 5.0):
            self.get_logger().info(
                f'Published {source} fix lat={lat:.8f} lon={lon:.8f} alt={alt:.2f}'
            )
            self._last_fix_log_time = now

    def _parse_lla_line(self, line: str):
        match = self._lla_pattern.search(line)
        if match is None:
            return None

        lat = self._parse_float(match.group('lat'))
        lon = self._parse_float(match.group('lon'))
        alt = self._parse_float(match.group('alt'))
        is_valid = not any(math.isnan(value) for value in (lat, lon, alt))
        return lat, lon, alt, is_valid, 'LLA'

    def _parse_nmea_line(self, line: str):
        if not line or line[0] != '$' or len(line) < 6:
            return None
        star_idx = line.rfind('*')
        if star_idx < 0 or (len(line) - star_idx) != 3:
            return None

        checksum_text = line[star_idx + 1:]
        try:
            expected_checksum = int(checksum_text, 16)
        except ValueError:
            return None

        actual_checksum = reduce(operator.xor, map(ord, line[1:star_idx]), 0)
        if actual_checksum != expected_checksum:
            return None

        body = line[1:star_idx]
        parts = body.split(',')
        message = parts[0]
        if message not in {'GPRMC', 'GNRMC'} or len(parts) < 7:
            return None

        if parts[2] == 'V':
            return float('nan'), float('nan'), float('nan'), False, message

        lat = self._nmea_to_degrees(parts[3], parts[4])
        lon = self._nmea_to_degrees(parts[5], parts[6])
        return lat, lon, float('nan'), True, message

    @staticmethod
    def _parse_float(value: str) -> float:
        text = str(value).strip()
        if text.lower() == 'nan':
            return float('nan')
        return float(text)

    @staticmethod
    def _nmea_to_degrees(gps_str: str, direction: str) -> float:
        if not gps_str or gps_str == '0':
            return float('nan')

        parts = gps_str.split('.')
        degrees_str = parts[0][:-2]
        minutes_str = parts[0][-2:]
        if len(parts) == 2:
            minutes_str += '.' + parts[1]

        degrees = float(degrees_str) if degrees_str else 0.0
        minutes = float(minutes_str) / 60.0 if minutes_str else 0.0
        value = degrees + minutes
        if direction in {'W', 'S'}:
            value *= -1.0
        return value

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSUdpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
