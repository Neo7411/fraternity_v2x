#!/usr/bin/env python3
# Szerzo: Angi David
# Unideb - Autoware & V2X Integracio
#
# Parameterezett valtozat: valodi ASN.1 UPER CAM + helyes GeoNetworking SHB
# keretezes. socktap NEM kell. Minden ROS parameter, az alapertekek a mar
# bevalt ertekek - igy parameterek nelkul is fut, de barmi felulirhato.
#
# Inditas (alapertekekkel):
#     sudo python3 autoware_cam_broadcaster.py
#
# Feluliras params fajllal (params.yaml):
#     /**:
#       ros__parameters:
#         origin_latitude: 47.5301
#         origin_longitude: 21.6243
#         station_id: 7
#         log_debug: true
#     sudo python3 autoware_cam_broadcaster.py --ros-args --params-file params.yaml
#
# Vagy egyetlen parameter felulirasa CLI-bol:
#     sudo python3 autoware_cam_broadcaster.py --ros-args -p station_id:=7 -p rate_hz:=2.0

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

try:
    import asn1tools
except ImportError:
    asn1tools = None


# ---- WGS84 ----
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

# 2004-01-01T00:00:00Z Unix-ms (ITS / TimestampIts epoch)
_ITS_EPOCH_MS = 1072915200000
ALT_UNAVAILABLE = 800001


def enu_to_geodetic(east, north, up, lat0_deg, lon0_deg, alt0):
    """Lokalis ENU eltolas (m) a (lat0,lon0,alt0) korul -> WGS84 lat/lon/alt.
    Ellipszoidi elsorendu kozelites; nehany km-en al-meteres. Nagy/MGRS terkepnel pyproj."""
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    d = 1.0 - _WGS84_E2 * s * s
    Rn = _WGS84_A / math.sqrt(d)
    Rm = _WGS84_A * (1.0 - _WGS84_E2) / (d ** 1.5)
    lat = lat0_deg + math.degrees(north / Rm)
    lon = lon0_deg + math.degrees(east / (Rn * math.cos(lat0)))
    return lat, lon, alt0 + up


def yaw_from_quaternion(x, y, z, w):
    """ENU yaw (rad), keletbol CCW pozitiv."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def its_timestamp_ms(offset_ms=0):
    return int(time.time() * 1000) - _ITS_EPOCH_MS + offset_ms


class AutowareCamBroadcaster(Node):
    def __init__(self):
        super().__init__('autoware_cam_broadcaster')

        # ---- parameterek (alapertekek = a bevalt ertekek) ----
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('vehicle_frame', 'base_link')
        self.declare_parameter('origin_latitude', 47.5301)
        self.declare_parameter('origin_longitude', 21.6243)
        self.declare_parameter('origin_altitude', 120.0)
        self.declare_parameter('interface', 'eth0')
        self.declare_parameter('dst_mac', 'ff:ff:ff:ff:ff:ff')      # GN broadcast
        self.declare_parameter('src_mac', '')                       # ures = az interface MAC-ja
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('asn1_files', [
            '/home/aw/vanetza_src/asn1/EN302637-2v141-CAM.asn',
            '/home/aw/vanetza_src/asn1/TS102894-2v131-CDD.asn',
        ])
        self.declare_parameter('station_id', 12345)
        self.declare_parameter('station_type', 5)                   # 5 = passengerCar
        self.declare_parameter('vehicle_length_dm', 49)             # 0.1 m (Lexus RX ~4.9 m)
        self.declare_parameter('vehicle_width_dm', 19)              # 0.1 m (~1.9 m)
        self.declare_parameter('gn_version', 1)
        self.declare_parameter('gn_lifetime_byte', 0x1a)
        self.declare_parameter('gn_traffic_class', 0x02)            # CAM = TC 2
        self.declare_parameter('timestamp_offset_ms', 0)           # TAI/leap korrekcio ha kell
        self.declare_parameter('position_confidence_cm', 100)      # placeholder
        self.declare_parameter('heading_confidence', 10)           # placeholder
        self.declare_parameter('speed_confidence', 10)             # placeholder
        self.declare_parameter('course_from', 'velocity')          # 'velocity' vagy 'orientation'
        self.declare_parameter('min_speed_for_course', 0.3)
        self.declare_parameter('speed_filter_alpha', 0.4)
        self.declare_parameter('log_debug', False)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.vehicle_frame = g('vehicle_frame').value
        self.lat0 = float(g('origin_latitude').value)
        self.lon0 = float(g('origin_longitude').value)
        self.alt0 = float(g('origin_altitude').value)
        self.interface = g('interface').value
        dst_mac_str = g('dst_mac').value
        src_mac_str = g('src_mac').value
        self.rate_hz = float(g('rate_hz').value)
        asn1_files = [p for p in g('asn1_files').value if p]
        self.station_id = int(g('station_id').value)
        self.station_type = int(g('station_type').value)
        self.veh_len_dm = int(g('vehicle_length_dm').value)
        self.veh_wid_dm = int(g('vehicle_width_dm').value)
        self.gn_version = int(g('gn_version').value)
        self.gn_lifetime = int(g('gn_lifetime_byte').value) & 0xFF
        self.gn_tc = int(g('gn_traffic_class').value) & 0xFF
        self.ts_offset = int(g('timestamp_offset_ms').value)
        self.pos_conf_cm = int(g('position_confidence_cm').value)
        self.heading_conf = int(g('heading_confidence').value)
        self.speed_conf = int(g('speed_confidence').value)
        self.course_from = g('course_from').value
        self.min_speed_for_course = float(g('min_speed_for_course').value)
        self.alpha = float(g('speed_filter_alpha').value)
        self.log_debug = bool(g('log_debug').value)

        # ---- ASN.1 (CAM rel1) ----
        if asn1tools is None:
            raise RuntimeError("Hianyzik az asn1tools.  pip install asn1tools")
        if not asn1_files:
            raise RuntimeError("Ures asn1_files. Add meg a CAM rel1 .asn fajlok utjat.")
        self.get_logger().info(f"ASN.1 forditas: {asn1_files}")
        self.cam_codec = asn1tools.compile_files(asn1_files, 'uper')

        # ---- nyers socket ----
        import socket
        self.dst_mac = bytes.fromhex(dst_mac_str.replace(':', ''))
        self.src_mac = (bytes.fromhex(src_mac_str.replace(':', ''))
                        if src_mac_str else self._read_iface_mac(self.interface))
        self.ethertype = 0x8947   # GeoNetworking
        try:
            self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
            self.sock.bind((self.interface, 0))
            self.get_logger().info(
                f"Nyers socket bindolva: {self.interface}, src MAC {self.src_mac.hex(':')}")
        except PermissionError:
            self.get_logger().error("Nincs jogosultsag a nyers sockethez! Futtasd sudo-val.")
            raise

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- sebesseg/irany allapot ----
        self._prev_en = None
        self._prev_t = None
        self._speed = 0.0
        self._course = 0.0
        self._have_course = False

        if self.lat0 == 0.0 and self.lon0 == 0.0:
            self.get_logger().warn(
                'origin_latitude/longitude == 0! Allitsd be (map_projector_info.yaml), '
                'kulonben rossz a CAM pozicio.')

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.broadcast_cam)
        self.get_logger().info(
            f"CAM broadcaster fut: {self.map_frame}->{self.vehicle_frame} @ {self.rate_hz} Hz")

    def _read_iface_mac(self, iface):
        with open(f'/sys/class/net/{iface}/address') as f:
            return bytes.fromhex(f.read().strip().replace(':', ''))

    # ---------------- fociklus ----------------
    def broadcast_cam(self):
        try:
            tr = self.tf_buffer.lookup_transform(self.map_frame, self.vehicle_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f"Meg nem elerheto a TF: {ex}", throttle_duration_sec=2.0)
            return

        t = tr.transform.translation
        r = tr.transform.rotation
        east, north, up = t.x, t.y, t.z

        stamp = tr.header.stamp
        t_now = stamp.sec + stamp.nanosec * 1e-9
        if self._prev_t is not None and t_now <= self._prev_t:
            t_now = self.get_clock().now().nanoseconds * 1e-9

        if self._prev_en is not None and self._prev_t is not None:
            dt = t_now - self._prev_t
            if dt > 1e-3:
                de = east - self._prev_en[0]
                dn = north - self._prev_en[1]
                inst = math.hypot(de, dn) / dt
                self._speed = self.alpha * inst + (1.0 - self.alpha) * self._speed
                if inst >= self.min_speed_for_course:
                    self._course = math.degrees(math.atan2(de, dn)) % 360.0
                    self._have_course = True
        self._prev_en = (east, north)
        self._prev_t = t_now

        if self.course_from == 'orientation' or not self._have_course:
            yaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)
            heading_deg = (90.0 - math.degrees(yaw)) % 360.0
        else:
            heading_deg = self._course

        lat, lon, alt = enu_to_geodetic(east, north, up, self.lat0, self.lon0, self.alt0)

        try:
            cam_payload = self.create_asn1_cam(lat, lon, alt, heading_deg, self._speed)
        except Exception as ex:   # noqa: BLE001
            self.get_logger().error(f"CAM ASN.1 kodolas sikertelen: {ex}", throttle_duration_sec=2.0)
            return

        lat_1e7 = int(round(lat * 1e7))
        lon_1e7 = int(round(lon * 1e7))
        frame = self.build_geonet_frame(cam_payload, lat_1e7, lon_1e7, self._speed, heading_deg)
        self.sock.send(frame)

        if self.log_debug:
            self.get_logger().info(
                f"CAM -> lat={lat:.7f} lon={lon:.7f} v={self._speed:.2f}m/s "
                f"hdg={heading_deg:.1f} ({len(cam_payload)}B CAM, {len(frame)}B frame)")

    # ---------------- CAM (ASN.1 UPER) ----------------
    def create_asn1_cam(self, lat, lon, alt, heading_deg, speed_mps):
        gen_delta = its_timestamp_ms(self.ts_offset) % 65536

        lat_1e7 = max(-900000000, min(900000000, int(round(lat * 1e7))))
        lon_1e7 = max(-1800000000, min(1800000000, int(round(lon * 1e7))))
        alt_cm = int(round(alt * 100))
        if not (-100000 <= alt_cm <= 800000):
            alt_cm = ALT_UNAVAILABLE
        heading_01 = int(round(heading_deg * 10)) % 3600
        speed_001 = max(0, min(16382, int(round(speed_mps * 100))))

        cam = {
            'header': {'protocolVersion': 2, 'messageID': 2, 'stationID': self.station_id},
            'cam': {
                'generationDeltaTime': gen_delta,
                'camParameters': {
                    'basicContainer': {
                        'stationType': self.station_type,
                        'referencePosition': {
                            'latitude': lat_1e7,
                            'longitude': lon_1e7,
                            'positionConfidenceEllipse': {
                                'semiMajorConfidence': self.pos_conf_cm,
                                'semiMinorConfidence': self.pos_conf_cm,
                                'semiMajorOrientation': 0,
                            },
                            'altitude': {
                                'altitudeValue': alt_cm,
                                'altitudeConfidence': 'unavailable',
                            },
                        },
                    },
                    'highFrequencyContainer': (
                        'basicVehicleContainerHighFrequency',
                        {
                            'heading': {'headingValue': heading_01,
                                        'headingConfidence': self.heading_conf},
                            'speed': {'speedValue': speed_001,
                                      'speedConfidence': self.speed_conf},
                            'driveDirection': 'forward',
                            'vehicleLength': {
                                'vehicleLengthValue': self.veh_len_dm,
                                'vehicleLengthConfidenceIndication': 'noTrailerPresent',
                            },
                            'vehicleWidth': self.veh_wid_dm,
                            'longitudinalAcceleration': {
                                'longitudinalAccelerationValue': 161,        # nem elerheto
                                'longitudinalAccelerationConfidence': 102,   # nem elerheto
                            },
                            'curvature': {'curvatureValue': 1023,
                                          'curvatureConfidence': 'unavailable'},
                            'curvatureCalculationMode': 'unavailable',
                            'yawRate': {'yawRateValue': 32767,
                                        'yawRateConfidence': 'unavailable'},
                        },
                    ),
                    # lowFrequencyContainer / specialVehicleContainer: OPTIONAL, most kihagyva.
                },
            },
        }
        return self.cam_codec.encode('CAM', cam)

    # ---------------- GeoNetworking SHB keretezes ----------------
    def _build_lpv(self, lat_1e7, lon_1e7, speed_mps, heading_deg):
        """Long Position Vector (24 bajt) - a forras pozicioja a GN retegben."""
        addr_conf = (0 << 15) | ((self.station_type & 0x1F) << 10) | 0
        gn_addr = struct.pack('!H', addr_conf) + self.src_mac
        tst = its_timestamp_ms(self.ts_offset) % (2 ** 32)
        pai = 1
        sfield = (pai << 15) | (int(round(speed_mps * 100)) & 0x7FFF)
        hval = int(round(heading_deg * 10)) % 3600
        return (gn_addr +
                struct.pack('!I', tst) +
                struct.pack('!i', lat_1e7) +
                struct.pack('!i', lon_1e7) +
                struct.pack('!H', sfield) +
                struct.pack('!H', hval))

    def build_geonet_frame(self, payload, lat_1e7, lon_1e7, speed_mps, heading_deg):
        eth = struct.pack('!6s6sH', self.dst_mac, self.src_mac, self.ethertype)
        b0 = ((self.gn_version & 0x0F) << 4) | 0x01
        gn_basic = struct.pack('!BBBB', b0, 0x00, self.gn_lifetime, 0x01)
        pl = 4 + len(payload)
        gn_common = struct.pack('!BBBBHBB', 0x20, 0x50, self.gn_tc, 0x80, pl, 0x01, 0x00)
        gn_ext = self._build_lpv(lat_1e7, lon_1e7, speed_mps, heading_deg) + bytes(4)
        btp_b = struct.pack('!HH', 2001, 0x0000)
        return eth + gn_basic + gn_common + gn_ext + btp_b + payload


def main(args=None):
    rclpy.init(args=args)
    node = AutowareCamBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.sock.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
