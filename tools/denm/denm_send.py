#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import json
import math
import time
import os 
import pyproj
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

import paho.mqtt.client as mqtt


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

    return pyproj.Transformer.from_crs(src, dst, always_xy=True)


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def enu_yaw_to_heading_deg(yaw_rad):
    """ENU yaw (kelettol, CCW) -> ETSI heading (eszaktol, oramutato szerint)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class AutowareDenmMqtt(Node):
    def __init__(self):
        super().__init__('autoware_denm_mqtt')

        p = self.declare_parameter
        self.mqtt_broker = p('mqtt_broker', '127.0.0.1').value
        self.mqtt_port = p('mqtt_port', 1883).value
        self.mqtt_topic = p('mqtt_topic', 'vanetza/in/denm').value

        self.station_id = os.environ["ROS_DOMAIN_ID"]
        self.station_type = int(p('station_type', 5).value)

        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.pose_source = p('pose_source', 'kinematic_state').value

        # DENM ismetles: 2 masodpercig, 10 Hz -> 20 uzenet
        self.rate_hz = float(p('rate_hz', 10.0).value)
        self.duration_s = float(p('duration_s', 2.0).value)

        # --- JSON SABLON BETOLTESE ---
        template_path = p('template_path', '/vanetza/examples/in_denm.json').value
        try:
            with open(template_path, 'r') as f:
                self.denm_template = json.load(f)
            self.get_logger().info(f"Sikeresen betoltve a sablon JSON: {template_path}")
        except Exception as e:
            self.get_logger().error(f"Hiba a JSON sablon betoltesekor: {e}")
            self.denm_template = {}

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

        # Egy esemenyhez egy actionId + egy detectionTime, az ismetlesek alatt fix
        self.sequence_number = 0
        self.detection_time = None
        self.sent_count = 0
        self.total_count = max(1, int(round(self.duration_s * self.rate_hz)))

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT Csatlakozva!')
        except Exception as e:
            self.get_logger().error(f'MQTT hiba: {e}')

        self.get_logger().info(
            f"DENM kuldes: {self.duration_s}s @ {self.rate_hz}Hz = {self.total_count} uzenet"
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _get_state(self):
        """Visszaadja: (e, n, u, yaw_rad, speed_mps), vagy None ha meg nincs pozicio.

        A yaw azert kell, mert a vevo oldal ebbol tudja meg, hogy szembe
        nezunk-e vele - allo jarmunel a poziciobol ez nem szamolhato ki.
        """
        speed = 0.0
        if self.last_odom is not None:
            tw = self.last_odom.twist.twist
            speed = math.hypot(tw.linear.x, tw.linear.y)

        if self.pose_source == 'tf':
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            except (LookupException, ExtrapolationException):
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return t.x, t.y, t.z, yaw, speed

        if self.last_odom is None:
            return None
        pp = self.last_odom.pose.pose
        yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y,
                          pp.orientation.z, pp.orientation.w)
        return pp.position.x, pp.position.y, pp.position.z, yaw, speed

    def _build_denm(self, lat, lon, alt):
        """A sablon szerkezetet valtozatlanul hagyja, csak az erteket irja at."""
        if not self.denm_template:
            return None

        # Memoriaban levo sablon teljes masolata, hogy ne irjuk felul az eredetit
        denm = copy.deepcopy(self.denm_template)
        now = time.time()

        try:
            mgmt = denm["management"]

            # --- FELULIRAS: ActionID (az egesz ismetlessorozatban ugyanaz) ---
            mgmt["actionId"]["originatingStationId"] = self.station_id
            mgmt["actionId"]["sequenceNumber"] = self.sequence_number

            # detectionTime = az esemeny keletkezese (fix), referenceTime = ez az uzenet
            mgmt["detectionTime"] = round(self.detection_time, 3)
            mgmt["referenceTime"] = round(now, 3)

            # --- FELULIRAS: eventPosition = az auto valos GPS poziciója ---
            ev = mgmt["eventPosition"]
            ev["latitude"] = lat
            ev["longitude"] = lon
            if "altitude" in ev:
                ev["altitude"]["altitudeValue"] = round(alt, 2)

            mgmt["validityDuration"] = int(math.ceil(self.duration_s))
            mgmt["stationType"] = self.station_type

        except KeyError as e:
            self.get_logger().error(f"Hianyzo kulcs a sablonban! Ellenorizd a JSON fajlt: {e}")
            return None

        return denm

    def _tick(self):
        st = self._get_state()
        if st is None:
            self.get_logger().warn('Nincs pozicio...', throttle_duration_sec=2.0)
            return

        e, n, u, yaw, speed = st
        lat, lon, alt = self.geo.convert(e, n, u)
        heading_deg = enu_yaw_to_heading_deg(yaw)

        # Az elso sikeres kuldesnel rogzitjuk az esemeny detektalasi idejet
        if self.detection_time is None:
            self.detection_time = time.time()

        denm = self._build_denm(lat, lon, alt)
        if denm is None:
            return

        if self.sent_count == 0:
            print("-" * 60 + "\n" + json.dumps(denm, indent=2) + "\n" + "-" * 60)

        self.mqtt_client.publish(self.mqtt_topic, json.dumps(denm))
        self.sent_count += 1
        self.get_logger().info(
            f"DENM {self.sent_count}/{self.total_count} kikuldve "
            f"(lat={lat:.7f}, lon={lon:.7f}, heading={heading_deg:.1f} fok, "
            f"v={speed:.1f} m/s)"
        )

        if self.sent_count >= self.total_count:
            self.get_logger().info('DENM ismetles kesz, leallas.')
            self.timer.cancel()
            raise SystemExit

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
    node = AutowareDenmMqtt()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
