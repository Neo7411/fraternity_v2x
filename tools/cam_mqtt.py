#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time
import copy  # Ehhez kell, hogy klónozzuk a sablont

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

import paho.mqtt.client as mqtt

# ITS epoch = 2004-01-01T00:00:00 UTC, UNIX ms
ITS_EPOCH_MS = 1072915200 * 1000

class EnuToWgs84:
    def __init__(self, lat0_deg, lon0_deg, h0_m):
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)
        self._lla2ecef = pyproj_transformer(4979, 4978)
        self._ecef2lla = pyproj_transformer(4978, 4979)
        self.x0, self.y0, self.z0 = self._lla2ecef.transform(lon0_deg, lat0_deg, h0_m)
        sl, cl = math.sin(self.lat0), math.cos(self.lat0)
        so, co = math.sin(self.lon0), math.cos(self.lon0)
        self.R = np.array([
            [-so, -sl * co, cl * co],
            [ co, -sl * so, cl * so],
            [0.0,  cl,      sl],
        ])

    def convert(self, e, n, u):
        ecef = np.array([self.x0, self.y0, self.z0]) + self.R @ np.array([e, n, u])
        lon, lat, h = self._ecef2lla.transform(ecef[0], ecef[1], ecef[2])
        return lat, lon, h

def pyproj_transformer(src, dst):
    import pyproj
    return pyproj.Transformer.from_crs(src, dst, always_xy=True)

def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def enu_yaw_to_heading_deg(yaw_rad):
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class AutowareCamMqtt(Node):
    def __init__(self):
        super().__init__('autoware_cam_mqtt')

        p = self.declare_parameter
        self.mqtt_broker = p('mqtt_broker', '127.0.0.1').value
        self.mqtt_port = p('mqtt_port', 1883).value
        self.mqtt_topic = p('mqtt_topic', 'vanetza/in/cam').value

        self.station_id = int(p('station_id', 1).value)
        self.station_type = int(p('station_type', 5).value)
        self.veh_len = float(p('vehicle_length_m', 0.0).value)
        self.veh_wid = float(p('vehicle_width_m', 0.0).value)

        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.pose_source = p('pose_source', 'kinematic_state').value
        self.rate_hz = float(p('rate_hz', 10.0).value)

        # --- JSON SABLON BETÖLTÉSE ---
        template_path = '/vanetza/examples/in_cam.json'
        try:
            with open(template_path, 'r') as f:
                self.cam_template = json.load(f)
            self.get_logger().info(f"Sikeresen betöltve a sablon JSON: {template_path}")
        except Exception as e:
            self.get_logger().error(f"Hiba a JSON sablon betöltésekor: {e}")
            self.cam_template = {} # Ha nem találja, üres marad és hiba lesz később

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = EnuToWgs84(lat0, lon0, alt0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.last_odom = None
        self.create_subscription(Odometry, '/localization/kinematic_state', self._on_odom, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._prev = None

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT Csatlakozva!')
        except Exception as e:
            self.get_logger().error(f'MQTT hiba: {e}')

        self.create_timer(1.0 / self.rate_hz, self._tick)

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _get_state(self):
        if self.pose_source == 'tf':
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            except (LookupException, ExtrapolationException):
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            e, n, u = t.x, t.y, t.z
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        else:
            if self.last_odom is None: return None
            pp = self.last_odom.pose.pose
            e, n, u = pp.position.x, pp.position.y, pp.position.z
            yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w)

        vx = vy = yaw_rate = None
        if self.last_odom is not None:
            tw = self.last_odom.twist.twist
            vx, vy, yaw_rate = tw.linear.x, tw.linear.y, tw.angular.z

        now = time.time()
        if vx is None:
            heading = enu_yaw_to_heading_deg(yaw)
            if self._prev is not None:
                dt = now - self._prev[0]
                if dt > 1e-3:
                    de, dn = e - self._prev[1], n - self._prev[2]
                    vx, vy = math.hypot(de, dn) / dt, 0.0
                    yaw_rate = math.radians((heading - self._prev[3] + 540) % 360 - 180) / dt
            self._prev = (now, e, n, heading)
            if vx is None: vx, vy, yaw_rate = 0.0, 0.0, 0.0

        return e, n, u, yaw, vx, vy, yaw_rate

    def _gen_delta_time(self):
        return int((time.time() * 1000 - ITS_EPOCH_MS)) % 65536

    def _build_cam(self, lat, lon, heading_deg, speed_mps, drive_dir, yaw_rate_dps):
        if not self.cam_template:
            return None

        # Memóriában lévő sablon teljes másolása, hogy ne írjuk felül az eredetit
        cam = copy.deepcopy(self.cam_template)

        veh_len = round(self.veh_len, 1) if self.veh_len > 0 else 1023
        veh_wid = round(self.veh_wid, 1) if self.veh_wid > 0 else 62

        try:
            # Rövidítések a mély struktúrákhoz
            basic_container = cam["camParameters"]["basicContainer"]
            ref_pos = basic_container["referencePosition"]
            hf_container = cam["camParameters"]["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]

            # --- FELÜLÍRÁS: BasicContainer ---
            basic_container["stationType"] = self.station_type
            ref_pos["latitude"] = lat
            ref_pos["longitude"] = lon

            # --- FELÜLÍRÁS: HighFrequencyContainer ---
            hf_container["heading"]["headingValue"] = round(heading_deg, 1)
            hf_container["speed"]["speedValue"] = round(max(0.0, speed_mps), 2)
            hf_container["driveDirection"] = drive_dir
            hf_container["yawRate"]["yawRateValue"] = round(yaw_rate_dps, 2)
            
            hf_container["vehicleLength"]["vehicleLengthValue"] = veh_len
            hf_container["vehicleWidth"] = veh_wid

            # --- FELÜLÍRÁS: Időbélyeg ---
            cam["generationDeltaTime"] = self._gen_delta_time()

        except KeyError as e:
            self.get_logger().error(f"Hiányzó kulcs a sablonban! Ellenőrizd a JSON fájlt: {e}")

        return cam

    def _tick(self):
        st = self._get_state()
        if st is None:
            self.get_logger().warn('Nincs pozicio...', throttle_duration_sec=2.0)
            return
        e, n, u, yaw, vx, vy, yaw_rate = st
        lat, lon, _ = self.geo.convert(e, n, u)
        heading_deg = enu_yaw_to_heading_deg(yaw)
        speed = math.hypot(vx, vy)
        drive_dir = 0 if vx >= 0 else 1
        yaw_rate_dps = math.degrees(yaw_rate)

        cam = self._build_cam(lat, lon, heading_deg, speed, drive_dir, yaw_rate_dps)
        if cam:
            self.mqtt_client.publish(self.mqtt_topic, json.dumps(cam))

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
    node = AutowareCamMqtt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()