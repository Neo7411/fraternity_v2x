#!/usr/bin/env python3

import math
import os
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import (
    PredictedObjects, PredictedObject, PredictedObjectKinematics,
    PredictedPath, ObjectClassification, Shape,
)

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

ETH_P_GEONET = 0x8947
BTP_PORT_CAM = 2001

# ETSI StationType -> autoware ObjectClassification.label
_STATIONTYPE_TO_LABEL = {
    0: ObjectClassification.UNKNOWN,
    1: ObjectClassification.PEDESTRIAN,
    2: ObjectClassification.BICYCLE,     # cyclist
    3: ObjectClassification.MOTORCYCLE,  # moped
    4: ObjectClassification.MOTORCYCLE,
    5: ObjectClassification.CAR,         # passengerCar
    6: ObjectClassification.BUS,
    7: ObjectClassification.TRUCK,       # lightTruck
    8: ObjectClassification.TRUCK,       # heavyTruck
    9: ObjectClassification.TRAILER,
}


# ============================================================================
# WGS84 <-> lokalis ENU
# ============================================================================
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


def geodetic_to_enu(lat, lon, alt, lat0_deg, lon0_deg, alt0):
    """WGS84 -> lokalis ENU (m) a (lat0,lon0,alt0) korul. Az enu_to_geodetic inverze."""
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    d = 1.0 - _WGS84_E2 * s * s
    Rn = _WGS84_A / math.sqrt(d)
    Rm = _WGS84_A * (1.0 - _WGS84_E2) / (d ** 1.5)
    north = math.radians(lat - lat0_deg) * Rm
    east = math.radians(lon - lon0_deg) * Rn * math.cos(lat0)
    return east, north, alt - alt0


def yaw_from_quaternion(x, y, z, w):
    """ENU yaw (rad), keletbol CCW pozitiv."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def yaw_to_quaternion(yaw_rad):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw_rad / 2.0), w=math.cos(yaw_rad / 2.0))


def its_timestamp_ms(offset_ms=0):
    return int(time.time() * 1000) - _ITS_EPOCH_MS + offset_ms


# ============================================================================
# Bejovo keret -> nyers CAM byte-ok
# ============================================================================
def parse_geonet_cam(frame):
    """Ethernet/GN/BTP szetszedese -> a CAM nyers (UPER) byte-jai, vagy None.
    Csak unsecured SHB + BTP-B CAM (port 2001) kereteket fogad el."""
    if len(frame) < 58:
        return None
    if struct.unpack('!H', frame[12:14])[0] != ETH_P_GEONET:
        return None
    # GN Basic (14..18): byte0 = version<<4 | NH ; NH=1 -> common (unsecured)
    if (frame[14] & 0x0F) != 1:
        return None  # secured packet (NH=2) - itt nem kezeljuk
    # GN Common (18..26)
    common = frame[18:26]
    if (common[0] >> 4) != 2:          # NH = BTP-B
        return None
    if common[1] != 0x50:              # HT/HST = TSB/SHB
        return None
    pl = struct.unpack('!H', common[4:6])[0]
    # SHB extended header = 28 bajt (LPV 24 + reserved 4) -> 26..54
    # BTP-B (54..58)
    btp_port = struct.unpack('!H', frame[54:56])[0]
    if btp_port != BTP_PORT_CAM:
        return None
    cam_len = max(0, pl - 4)
    cam = frame[58:58 + cam_len] if cam_len else frame[58:]
    return cam if cam else None


# ============================================================================
# CAM node
# ============================================================================
class AutowareCamNode(Node):
    def __init__(self):
        super().__init__('autoware_cam_node')

        # ---- parameterek: NATIV ROS, default NELKUL (minden a params.yaml-bol) ----
        # A tipus megadasaval (Parameter.Type.*) a parameter KOTELEZO: ha hianyzik
        # a params fajlbol, a node ParameterUninitializedException-nel elszall.
        T = Parameter.Type

        def decl(name, ptype):
            self.declare_parameter(name, ptype)
            return self.get_parameter(name).value

        # kozos
        self.interface = decl('interface', T.STRING)
        self.lat0 = float(decl('origin_latitude', T.DOUBLE))
        self.lon0 = float(decl('origin_longitude', T.DOUBLE))
        self.alt0 = float(decl('origin_altitude', T.DOUBLE))
        self.log_debug = bool(decl('log_debug', T.BOOL))
        asn1_files = [x for x in decl('asn1_files', T.STRING_ARRAY) if x]
        # kuldes
        self.map_frame = decl('map_frame', T.STRING)
        self.vehicle_frame = decl('vehicle_frame', T.STRING)
        dst_mac_str = decl('dst_mac', T.STRING)
        src_mac_str = decl('src_mac', T.STRING)
        self.rate_hz = float(decl('rate_hz', T.DOUBLE))
        self.station_id = int(decl('station_id', T.INTEGER))
        # Ha a station_id 12345 (a yaml-ban hagyott alapertek) es van AWID env
        # (a run_cont.sh allitja kontenerenkent egyedire), abbol generaljuk.
        # Igy minden konteneren MAS station_id lesz -> a fogado nem dobja el a
        # masik CAM-jat sajatkent. KULONBEN nem latszanak a PredictedObjects-ek!
        awid = os.environ.get('AWID', '').strip()
        if self.station_id == 12345 and awid.isdigit():
            self.station_id = 10000 + int(awid)
        self.station_type = int(decl('station_type', T.INTEGER))
        self.veh_len_dm = int(decl('vehicle_length_dm', T.INTEGER))
        self.veh_wid_dm = int(decl('vehicle_width_dm', T.INTEGER))
        self.gn_version = int(decl('gn_version', T.INTEGER))
        self.gn_lifetime = int(decl('gn_lifetime_byte', T.INTEGER)) & 0xFF
        self.gn_tc = int(decl('gn_traffic_class', T.INTEGER)) & 0xFF
        self.ts_offset = int(decl('timestamp_offset_ms', T.INTEGER))
        self.pos_conf_cm = int(decl('position_confidence_cm', T.INTEGER))
        self.heading_conf = int(decl('heading_confidence', T.INTEGER))
        self.speed_conf = int(decl('speed_confidence', T.INTEGER))
        self.course_from = decl('course_from', T.STRING)
        self.min_speed_for_course = float(decl('min_speed_for_course', T.DOUBLE))
        self.alpha = float(decl('speed_filter_alpha', T.DOUBLE))
        # fogadas
        out_topic = decl('output_topic', T.STRING)
        self.frame_id = decl('frame_id', T.STRING)
        recv_rate_hz = float(decl('publish_rate_hz', T.DOUBLE))
        self.timeout_s = float(decl('object_timeout_s', T.DOUBLE))
        self.obj_h = float(decl('object_height', T.DOUBLE))
        self.path_horizon = float(decl('predicted_path_horizon_s', T.DOUBLE))
        self.path_dt = float(decl('predicted_path_dt_s', T.DOUBLE))

        # ---- ASN.1 (CAM rel1) - kozos codec kuldeshez es fogadashoz ----
        if asn1tools is None:
            raise RuntimeError("Hianyzik az asn1tools.  pip install asn1tools")
        if not asn1_files:
            raise RuntimeError("Ures asn1_files. Add meg a CAM rel1 .asn fajlok utjat.")
        self.get_logger().info(f"ASN.1 forditas: {asn1_files}")
        self.cam_codec = asn1tools.compile_files(asn1_files, 'uper')

        if self.lat0 == 0.0 and self.lon0 == 0.0:
            self.get_logger().warn(
                'origin_latitude/longitude == 0! Allitsd be (map_projector_info.yaml), '
                'kulonben rossz a CAM pozicio.')

        # ---- nyers socket (kuldes ES fogadas ugyanazon a leiron) ----
        self.dst_mac = bytes.fromhex(dst_mac_str.replace(':', ''))
        self.src_mac = (bytes.fromhex(src_mac_str.replace(':', ''))
                        if src_mac_str else self._read_iface_mac(self.interface))
        self.ethertype = ETH_P_GEONET
        try:
            self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                                      socket.htons(ETH_P_GEONET))
            self.sock.bind((self.interface, ETH_P_GEONET))
            self.sock.settimeout(0.5)
            self.get_logger().info(
                f"Nyers socket bindolva: {self.interface}, src MAC {self.src_mac.hex(':')}")
        except PermissionError:
            self.get_logger().error("Nincs jogosultsag a nyers sockethez! Futtasd sudo-val.")
            raise

        # ---- TF (kuldeshez) ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- sebesseg/irany allapot (kuldeshez) ----
        self._prev_en = None
        self._prev_t = None
        self._speed = 0.0
        self._course = 0.0
        self._have_course = False

        # ---- fogadott objektumok: stationID -> dict(...) ----
        self._objects = {}
        self._lock = threading.Lock()

        # ---- ROS interfeszek ----
        self.pub = self.create_publisher(PredictedObjects, out_topic, 10)
        self.send_timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.broadcast_cam)
        self.pub_timer = self.create_timer(1.0 / max(recv_rate_hz, 0.1), self._publish)

        # ---- vevo szal ----
        threading.Thread(target=self._recv_loop, daemon=True).start()

        self.get_logger().info(
            f"CAM node fut: kuldes {self.map_frame}->{self.vehicle_frame} @ {self.rate_hz} Hz, "
            f"fogadas -> {out_topic} (frame '{self.frame_id}'), station_id={self.station_id}")

    def _read_iface_mac(self, iface):
        with open(f'/sys/class/net/{iface}/address') as f:
            return bytes.fromhex(f.read().strip().replace(':', ''))

    # ========================================================================
    # KULDES
    # ========================================================================
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

    # ========================================================================
    # FOGADAS
    # ========================================================================
    def _recv_loop(self):
        while rclpy.ok():
            try:
                frame = self.sock.recv(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            cam_bytes = parse_geonet_cam(frame)
            if not cam_bytes:
                continue
            try:
                cam = self.cam_codec.decode('CAM', cam_bytes)
            except Exception:   # noqa: BLE001 - hibas/idegen csomag
                continue
            self._handle_cam(cam)

    def _handle_cam(self, cam):
        sid = cam['header']['stationID']
        if sid == self.station_id:
            return  # sajat CAM kihagyasa
        try:
            params = cam['cam']['camParameters']
            bc = params['basicContainer']
            ref = bc['referencePosition']
            lat_1e7 = ref['latitude']
            lon_1e7 = ref['longitude']
            if lat_1e7 == 900000001 or lon_1e7 == 1800000001:
                return  # nincs ervenyes pozicio
            lat = lat_1e7 / 1e7
            lon = lon_1e7 / 1e7
            alt_v = ref['altitude']['altitudeValue']
            alt = (alt_v / 100.0) if -100000 <= alt_v <= 800000 else self.alt0
            station_type = bc.get('stationType', 5)

            choice, hf = params['highFrequencyContainer']
            heading_deg = 0.0
            speed_mps = 0.0
            veh_len = 4.5
            veh_wid = 1.9
            if choice == 'basicVehicleContainerHighFrequency':
                hv = hf['heading']['headingValue']
                heading_deg = (hv / 10.0) if hv != 3601 else 0.0
                sv = hf['speed']['speedValue']
                speed_mps = (sv / 100.0) if sv != 16383 else 0.0
                lv = hf['vehicleLength']['vehicleLengthValue']
                if 1 <= lv < 1023:
                    veh_len = lv / 10.0
                wv = hf['vehicleWidth']
                if 1 <= wv < 62:
                    veh_wid = wv / 10.0
        except (KeyError, TypeError, ValueError):
            return

        east, north, up = geodetic_to_enu(lat, lon, alt, self.lat0, self.lon0, self.alt0)
        yaw = math.radians(90.0 - heading_deg)   # kompasz (eszaktol CW) -> ENU yaw (keletbol CCW)
        label = _STATIONTYPE_TO_LABEL.get(station_type, ObjectClassification.UNKNOWN)

        with self._lock:
            self._objects[sid] = {
                'east': east, 'north': north, 'up': up, 'yaw': yaw,
                'speed': speed_mps, 'len': veh_len, 'wid': veh_wid,
                'label': label, 'last_seen': time.time(),
            }
        if self.log_debug:
            self.get_logger().info(
                f"CAM sid={sid} -> map x={east:.1f} y={north:.1f} "
                f"yaw={math.degrees(yaw):.0f} v={speed_mps:.1f} m/s")

    # ---------------- publikalas (RViz) ----------------
    def _publish(self):
        now = time.time()
        msg = PredictedObjects()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        with self._lock:
            stale = [sid for sid, o in self._objects.items()
                     if now - o['last_seen'] > self.timeout_s]
            for sid in stale:
                del self._objects[sid]
            snapshot = list(self._objects.items())

        for sid, o in snapshot:
            msg.objects.append(self._make_object(sid, o))
        self.pub.publish(msg)

    def _make_object(self, sid, o):
        obj = PredictedObject()

        uid = UUID()
        uid.uuid = list(struct.pack('!I', sid & 0xFFFFFFFF) + b'\x00' * 12)
        obj.object_id = uid
        obj.existence_probability = 1.0

        cls = ObjectClassification()
        cls.label = o['label']
        cls.probability = 1.0
        obj.classification = [cls]

        kin = PredictedObjectKinematics()
        pose = kin.initial_pose_with_covariance.pose
        pose.position = Point(x=o['east'], y=o['north'], z=o['up'])
        pose.orientation = yaw_to_quaternion(o['yaw'])
        # twist a jarmu sajat koordinatajaban (x elore)
        kin.initial_twist_with_covariance.twist.linear.x = o['speed']

        # egyszeru egyenes vonalu predikcio a jelenlegi sebesseg/irany alapjan
        path = PredictedPath()
        n = max(1, int(self.path_horizon / max(self.path_dt, 1e-3)))
        vx = o['speed'] * math.cos(o['yaw'])
        vy = o['speed'] * math.sin(o['yaw'])
        for i in range(n + 1):
            t = i * self.path_dt
            p = Point(x=o['east'] + vx * t, y=o['north'] + vy * t, z=o['up'])
            path.path.append(Pose(position=p, orientation=yaw_to_quaternion(o['yaw'])))
        sec = int(self.path_dt)
        path.time_step = Duration(sec=sec, nanosec=int((self.path_dt - sec) * 1e9))
        path.confidence = 1.0
        kin.predicted_paths = [path]
        obj.kinematics = kin

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=o['len'], y=o['wid'], z=self.obj_h)
        obj.shape = shape
        return obj


def main(args=None):
    rclpy.init(args=args)
    node = AutowareCamNode()
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
