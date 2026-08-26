#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import struct
import threading

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import paho.mqtt.client as mqtt

from geometry_msgs.msg import Quaternion, Point, Vector3
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import (
    TrackedObjects, TrackedObject, TrackedObjectKinematics,
    ObjectClassification, Shape,
)

VERSION = '2026-08-26 z-fix'

V2X_TAG = b'V2X'  # 0x56 0x32 0x58


def station_id_to_uuid(station_id):
    """UUID: [stationID 4 byte big-endian][b'V2X'][9 x 0x00]"""
    u = UUID()
    u.uuid = list(struct.pack('>I', int(station_id) & 0xFFFFFFFF) + V2X_TAG + bytes(9))
    return u


def uuid_to_station_id(u):
    """Visszafejtes: melyik ITS-S kuldte az objektumot."""
    return struct.unpack('>I', bytes(u.uuid[:4]))[0]


def is_v2x_uuid(u):
    """True, ha V2X-bol jott, nem lokalis erzekelesbol."""
    return bytes(u.uuid[4:7]) == V2X_TAG


def clamp(value, lo, hi, default):
    """Tartomanyon kivuli erteket alapertelmezesre cserel."""
    v = float(value)
    return v if lo <= v <= hi else default


class Wgs84ToEnu:
    def __init__(self, lat0_deg, lon0_deg, h0_m):
        self._lla2ecef = pyproj.Transformer.from_crs(4979, 4978, always_xy=True)
        self.x0, self.y0, self.z0 = self._lla2ecef.transform(lon0_deg, lat0_deg, h0_m)
        sl, cl = math.sin(math.radians(lat0_deg)), math.cos(math.radians(lat0_deg))
        so, co = math.sin(math.radians(lon0_deg)), math.cos(math.radians(lon0_deg))
        self.R = np.array([
            [-so,      co,      0.0],
            [-sl * co, -sl * so, cl],
            [cl * co,  cl * so,  sl],
        ])

    def convert(self, lat_deg, lon_deg, h_m):
        x, y, z = self._lla2ecef.transform(lon_deg, lat_deg, h_m)
        enu = self.R @ np.array([x - self.x0, y - self.y0, z - self.z0])
        return float(enu[0]), float(enu[1]), float(enu[2])


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def heading_deg_to_enu_yaw(heading_deg):
    return math.radians(90.0 - heading_deg)


class CAMToTracked(Node):
    def __init__(self):
        super().__init__('cam_mqtt_to_tracked')

        p = self.declare_parameter
        self.mqtt_broker = p('mqtt_broker', '127.0.0.1').value
        self.mqtt_port = int(p('mqtt_port', 1883).value)
        self.cam_topic = p('mqtt_cam_topic', 'vanetza/out/cam').value

        self.out_topic = p('output_topic', '/perception/object_recognition/tracking/objects').value
        self.map_frame = p('map_frame', 'map').value

        # 50 Hz: erre a topicra a multi_object_tracker is publikal ures listat
        self.rate_hz = float(p('rate_hz', 50.0).value)
        self.timeout_s = float(p('object_timeout_s', 3.0).value)

        # A kuldo nem irja felul az altitudeValue-t, csak a sablonbol orokli,
        # ezert alapbol fix talajszintre tesszuk az objektumokat.
        self.use_cam_altitude = bool(p('use_cam_altitude', False).value)
        self.object_z = float(p('object_z', 0.0).value)

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        self.alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = Wgs84ToEnu(lat0, lon0, self.alt0)

        self._objects = {}
        self._lock = threading.Lock()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(TrackedObjects, self.out_topic, qos)

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

        self.create_timer(1.0 / self.rate_hz, self._publish_tick)
        self.get_logger().info(
            f"[{VERSION}] '{self.cam_topic}' -> '{self.out_topic}' "
            f'({self.rate_hz:.0f} Hz), objektum z={self.object_z:.1f} m')

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe(self.cam_topic)
        self.get_logger().info(f'MQTT csatlakozva, feliratkozva: {self.cam_topic}')

    def _on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode('utf-8'))

        sid = int(data['fields']['header']['stationId'])

        cam = data['fields']['cam']['camParameters']
        ref = cam['basicContainer']['referencePosition']
        hf = cam['highFrequencyContainer']['basicVehicleContainerHighFrequency']

        lat = float(ref['latitude'])
        lon = float(ref['longitude'])
        alt = clamp(ref['altitude']['altitudeValue'], -1000.0, 9000.0, self.alt0)

        heading = clamp(hf['heading']['headingValue'], 0.0, 360.0, 0.0)
        speed = clamp(hf['speed']['speedValue'], 0.0, 90.0, 0.0)
        yaw_rate = clamp(hf['yawRate']['yawRateValue'], -180.0, 180.0, 0.0)
        length = clamp(hf['vehicleLength']['vehicleLengthValue'], 0.5, 25.0, 4.5)
        width = clamp(hf['vehicleWidth'], 0.4, 4.0, 1.8)

        e, n, u = self.geo.convert(lat, lon, alt)
        if not self.use_cam_altitude:
            u = self.object_z

        with self._lock:
            self._objects[sid] = {
                'e': e, 'n': n, 'u': u,
                'yaw': heading_deg_to_enu_yaw(heading),
                'speed': speed,
                'yaw_rate': math.radians(yaw_rate),
                'length': length, 'width': width,
                'stamp': self.get_clock().now(),
            }

        self.get_logger().info(
            f'Auto (ID: {sid}) pozicio: E={e:.2f}, N={n:.2f}, Z={u:.2f}, '
            f'Irany: {heading:.1f} deg, v={speed:.2f} m/s',
            throttle_duration_sec=1.0)

    def _build_tracked_object(self, sid, s):
        obj = TrackedObject()
        obj.object_id = station_id_to_uuid(sid)
        obj.existence_probability = 1.0

        cls = ObjectClassification()
        cls.label = ObjectClassification.CAR
        cls.probability = 1.0
        obj.classification = [cls]

        kin = TrackedObjectKinematics()
        kin.pose_with_covariance.pose.position = Point(x=s['e'], y=s['n'], z=s['u'])
        kin.pose_with_covariance.pose.orientation = yaw_to_quaternion(s['yaw'])
        cov = [0.0] * 36
        cov[0], cov[7], cov[14], cov[35] = 0.5, 0.5, 1.0, 0.2
        kin.pose_with_covariance.covariance = cov
        kin.twist_with_covariance.twist.linear.x = s['speed']
        kin.twist_with_covariance.twist.angular.z = s['yaw_rate']
        kin.orientation_availability = TrackedObjectKinematics.AVAILABLE
        kin.is_stationary = bool(s['speed'] < 0.1)
        obj.kinematics = kin

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=s['length'], y=s['width'], z=1.5)
        obj.shape = shape
        return obj

    def _publish_tick(self):
        now = self.get_clock().now()
        msg = TrackedObjects()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame

        with self._lock:
            alive = {}
            for sid, s in self._objects.items():
                if (now - s['stamp']).nanoseconds * 1e-9 > self.timeout_s:
                    self.get_logger().info(f'Allomas elveszett (ID: {sid})')
                    continue
                alive[sid] = s
                msg.objects.append(self._build_tracked_object(sid, s))
            self._objects = alive

        self.pub.publish(msg)

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = CAMToTracked()
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