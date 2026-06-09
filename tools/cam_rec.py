#!/usr/bin/env python3

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import (
    PredictedObjects, PredictedObject, PredictedObjectKinematics,
    PredictedPath, ObjectClassification, Shape,
)

try:
    import asn1tools
except ImportError:
    asn1tools = None


# ---- WGS84 ----
_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)

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


def geodetic_to_enu(lat, lon, alt, lat0_deg, lon0_deg, alt0):
    """WGS84 -> lokalis ENU (m) a (lat0,lon0,alt0) korul. A kuldo enu_to_geodetic inverze."""
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    d = 1.0 - _WGS84_E2 * s * s
    Rn = _WGS84_A / math.sqrt(d)
    Rm = _WGS84_A * (1.0 - _WGS84_E2) / (d ** 1.5)
    north = math.radians(lat - lat0_deg) * Rm
    east = math.radians(lon - lon0_deg) * Rn * math.cos(lat0)
    return east, north, alt - alt0


def yaw_to_quaternion(yaw_rad):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw_rad / 2.0), w=math.cos(yaw_rad / 2.0))


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


class AutowareCamReceiver(Node):
    def __init__(self):
        super().__init__('autoware_cam_receiver')

        self.declare_parameter('interface', 'eth0')
        self.declare_parameter('output_topic', '/v2x/cam/objects')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('asn1_files', [
            '/home/aw/vanetza_src/asn1/EN302637-2v141-CAM.asn',
            '/home/aw/vanetza_src/asn1/TS102894-2v131-CDD.asn',
        ])
        # UGYANAZ mint a kuldonel:
        self.declare_parameter('origin_latitude', 47.5301)
        self.declare_parameter('origin_longitude', 21.6243)
        self.declare_parameter('origin_altitude', 120.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('object_timeout_s', 2.0)     # ennyi ido utan eltunik az objektum
        self.declare_parameter('object_height', 1.5)        # m, a CAM-ben nincs magassag
        self.declare_parameter('predicted_path_horizon_s', 5.0)
        self.declare_parameter('predicted_path_dt_s', 0.5)
        self.declare_parameter('own_station_id', -1)        # >=0 -> sajat CAM kihagyasa
        self.declare_parameter('log_debug', False)

        g = self.get_parameter
        self.interface = g('interface').value
        self.frame_id = g('frame_id').value
        self.lat0 = float(g('origin_latitude').value)
        self.lon0 = float(g('origin_longitude').value)
        self.alt0 = float(g('origin_altitude').value)
        self.timeout_s = float(g('object_timeout_s').value)
        self.obj_h = float(g('object_height').value)
        self.path_horizon = float(g('predicted_path_horizon_s').value)
        self.path_dt = float(g('predicted_path_dt_s').value)
        self.own_station_id = int(g('own_station_id').value)
        self.log_debug = bool(g('log_debug').value)
        out_topic = g('output_topic').value
        rate_hz = float(g('publish_rate_hz').value)
        asn1_files = [p for p in g('asn1_files').value if p]

        if asn1tools is None:
            raise RuntimeError("Hianyzik az asn1tools.  pip install asn1tools")
        if not asn1_files:
            raise RuntimeError("Ures asn1_files. Add meg a CAM rel1 .asn fajlok utjat.")
        self.cam_codec = asn1tools.compile_files(asn1_files, 'uper')

        self.pub = self.create_publisher(PredictedObjects, out_topic, 10)

        # stationID -> dict(east,north,up,yaw,speed,len,wid,label,last_seen)
        self._objects = {}
        self._lock = threading.Lock()

        # ---- vevo socket ----
        try:
            self.sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_GEONET))
            self.sock.bind((self.interface, ETH_P_GEONET))
            self.sock.settimeout(0.5)
        except PermissionError:
            self.get_logger().error("Nincs jogosultsag a nyers sockethez! Futtasd sudo-val.")
            raise

        threading.Thread(target=self._recv_loop, daemon=True).start()
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._publish)
        self.get_logger().info(
            f"CAM vevo fut: {self.interface} -> {out_topic} (frame '{self.frame_id}')")

    # ---------------- vevo szal ----------------
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
        if self.own_station_id >= 0 and sid == self.own_station_id:
            return
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

    # ---------------- publikalas ----------------
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
    node = AutowareCamReceiver()
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
