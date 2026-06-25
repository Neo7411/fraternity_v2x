#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import struct
import threading

import numpy as np
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

def tf(src, dst):
    import pyproj
    return pyproj.Transformer.from_crs(src, dst, always_xy=True)

class Wgs84ToEnu:
    def __init__(self, lat0_deg, lon0_deg, h0_m):
        self._lla2ecef = tf(4979, 4978)
        self.x0, self.y0, self.z0 = self._lla2ecef.transform(lon0_deg, lat0_deg, h0_m)
        sl, cl = math.sin(math.radians(lat0_deg)), math.cos(math.radians(lat0_deg))
        so, co = math.sin(math.radians(lon0_deg)), math.cos(math.radians(lon0_deg))
        self.R = np.array([
            [-so,     co,      0.0],
            [-sl * co, -sl * so, cl],
            [ cl * co,  cl * so, sl],
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

def station_id_to_uuid(station_id):
    u = UUID()
    packed = struct.pack('>Q', int(station_id) & 0xFFFFFFFFFFFFFFFF)
    u.uuid = list(packed) + [0] * 8
    return u

LAT_UNAVAIL = 900000001
LON_UNAVAIL = 1800000001
HEADING_UNAVAIL = 3601
SPEED_UNAVAIL = 16383
LEN_UNAVAIL = 1023
WIDTH_UNAVAIL = 62
YAWRATE_UNAVAIL = 32767
ALT_UNAVAIL = 800001

class CAMToTracked(Node):
    def __init__(self):
        super().__init__('cam_mqtt_to_tracked')

        p = self.declare_parameter
        self.mqtt_broker = p('mqtt_broker', '127.0.0.1').value
        self.mqtt_port = int(p('mqtt_port', 1883).value)
        self.cam_topic = p('mqtt_cam_topic', 'vanetza/out/cam').value

        self.out_topic = p('output_topic', '/perception/object_recognition/tracking/objects').value
        self.map_frame = p('map_frame', 'map').value
        
        # 20 Hz-es frissítés az RViz stabilitásáért (megszűnik a villogás)
        self.rate_hz = float(p('rate_hz', 30.0).value)
        self.timeout_s = float(p('object_timeout_s', 3.0).value)

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = Wgs84ToEnu(lat0, lon0, alt0)
        self.alt0 = alt0

        self._objects = {}
        self._lock = threading.Lock()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(TrackedObjects, self.out_topic, qos)

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT csatlakozasi hiba: {e}')

        self.create_timer(1.0 / self.rate_hz, self._publish_tick)
        self.get_logger().info(f"Figyelem: '{self.cam_topic}' -> '{self.out_topic}' (20 Hz)")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            client.subscribe(self.cam_topic)
            self.get_logger().info(f"MQTT csatlakozva, feliratkozva: {self.cam_topic}")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except Exception:
            return

        sid = data.get('stationID')
        if sid is None:
            sid = data.get('fields', {}).get('header', {}).get('stationId')
        if sid is None:
            return

        cam_params = data.get('fields', {}).get('cam', {}).get('camParameters', {})
        if not cam_params:
            cam_params = data.get('camParameters', {})

        lat, lon, alt = LAT_UNAVAIL, LON_UNAVAIL, ALT_UNAVAIL
        heading, speed, yaw_rate = HEADING_UNAVAIL, SPEED_UNAVAIL, YAWRATE_UNAVAIL
        length, width = LEN_UNAVAIL, WIDTH_UNAVAIL

        if cam_params:
            basic = cam_params.get('basicContainer', {})
            ref_pos = basic.get('referencePosition', {})
            hf = cam_params.get('highFrequencyContainer', {}).get('basicVehicleContainerHighFrequency', {})

            lat = ref_pos.get('latitude', LAT_UNAVAIL)
            lon = ref_pos.get('longitude', LON_UNAVAIL)
            alt_obj = ref_pos.get('altitude', {})
            alt = alt_obj.get('altitudeValue', ALT_UNAVAIL) if isinstance(alt_obj, dict) else alt_obj
            
            hdg_obj = hf.get('heading', {})
            heading = hdg_obj.get('headingValue', HEADING_UNAVAIL) if isinstance(hdg_obj, dict) else hdg_obj
            spd_obj = hf.get('speed', {})
            speed = spd_obj.get('speedValue', SPEED_UNAVAIL) if isinstance(spd_obj, dict) else spd_obj
            yr_obj = hf.get('yawRate', {})
            yaw_rate = yr_obj.get('yawRateValue', YAWRATE_UNAVAIL) if isinstance(yr_obj, dict) else yr_obj
            
            len_obj = hf.get('vehicleLength', {})
            length = len_obj.get('vehicleLengthValue', LEN_UNAVAIL) if isinstance(len_obj, dict) else len_obj
            width = hf.get('vehicleWidth', WIDTH_UNAVAIL)

        if lat == LAT_UNAVAIL or lon == LON_UNAVAIL:
            return

        # OKOS VISSZASKÁLÁZÁS
        lat = float(lat)
        lon = float(lon)
        if abs(lat) > 900: lat /= 1e7
        if abs(lon) > 1800: lon /= 1e7

        alt = float(alt)
        if alt != ALT_UNAVAIL and abs(alt) > 1000: alt /= 100.0
        elif alt == ALT_UNAVAIL: alt = self.alt0

        # JAVÍTÁS: A heading már normál fokban van, NEM osztjuk 10-zel!
        heading = float(heading)
        
        speed = float(speed)
        if speed > 200 and speed != SPEED_UNAVAIL: speed /= 100.0 
        
        yaw_rate = float(yaw_rate)
        if abs(yaw_rate) > 500 and yaw_rate != YAWRATE_UNAVAIL: yaw_rate /= 100.0
        
        length = float(length)
        if length != LEN_UNAVAIL:
            if length >= 10: length /= 10.0
        else: length = 4.5
        
        width = float(width)
        if width != WIDTH_UNAVAIL:
            if width >= 10: width /= 10.0
        else: width = 1.8

        # ENU és YAW konverzió
        e, n, u = self.geo.convert(lat, lon, alt)
        yaw = heading_deg_to_enu_yaw(heading) if heading != HEADING_UNAVAIL else 0.0
        has_orientation = heading != HEADING_UNAVAIL
        speed = 0.0 if speed == SPEED_UNAVAIL else speed
        yaw_rate_rad = 0.0 if yaw_rate == YAWRATE_UNAVAIL else math.radians(yaw_rate)

        with self._lock:
            self._objects[int(sid)] = {
                'e': e, 'n': n, 'u': u,
                'yaw': yaw, 'has_orientation': has_orientation,
                'speed': speed, 'yaw_rate': yaw_rate_rad,
                'length': length, 'width': width,
                'stamp': self.get_clock().now(),
            }

        self.get_logger().info(f"Auto (ID: {sid}) pozicio: E={e:.2f}, N={n:.2f}, Igeny: {heading:.1f} deg")

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
        kin.orientation_availability = (TrackedObjectKinematics.AVAILABLE if s['has_orientation'] else TrackedObjectKinematics.UNAVAILABLE)
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
            stale = []
            for sid, s in self._objects.items():
                age = (now - s['stamp']).nanoseconds * 1e-9
                if age > self.timeout_s: stale.append(sid)
                else: msg.objects.append(self._build_tracked_object(sid, s))
            for sid in stale: del self._objects[sid]

        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass
        finally:
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()