#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EEBL DENM kuldo: vészfék + DENM. Beallitasok a fajl tetejen."""

import copy
import json
import math
import time

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import paho.mqtt.client as mqtt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from tier4_external_api_msgs.srv import SetEmergency

# ---------------- BEALLITASOK ----------------
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "vanetza/in/denm"
TEMPLATE = "/vanetza/examples/in_denm.json"

MAP_ORIGIN = (47.5316, 21.6273, 0.0)     # lat, lon, alt
DECEL_THRESHOLD = -3.0                    # m/s^2, ennel indul az esemeny
RATE_HZ = 10.0
DURATION_S = 2.0
STATION_TYPE = 5
# ---------------------------------------------


class Geo:
    def __init__(self, lat0, lon0, h0):
        self.f_in = pyproj.Transformer.from_crs(4979, 4978, always_xy=True)
        self.f_out = pyproj.Transformer.from_crs(4978, 4979, always_xy=True)
        self.o = np.array(self.f_in.transform(lon0, lat0, h0))
        la, lo = math.radians(lat0), math.radians(lon0)
        sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        self.R = np.array([[-so, -sl * co, cl * co],
                           [co, -sl * so, cl * so],
                           [0.0, cl, sl]])

    def to_wgs(self, e, n, u):
        x, y, z = self.o + self.R @ np.array([e, n, u])
        lon, lat, h = self.f_out.transform(x, y, z)
        return lat, lon, h


def info_quality(decel):
    a = abs(decel)
    for lim, q in ((1, 1), (2, 2), (3, 3), (4.5, 4), (6, 5), (8, 6)):
        if a < lim:
            return q
    return 7


class EeblSender(Node):
    def __init__(self):
        super().__init__("eebl_sender")

        with open(TEMPLATE) as f:
            self.template = json.load(f)

        self.geo = Geo(*MAP_ORIGIN)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt.loop_start()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.odom = None
        self.accel = 0.0
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._odom_cb, qos)
        self.create_subscription(AccelWithCovarianceStamped,
                                 "/localization/acceleration", self._accel_cb, qos)

        self.cli = self.create_client(SetEmergency, "/api/autoware/set/emergency")
        self.braked = False
        self.event = None
        self.peak = 0.0
        self.sent = 0
        self.total = int(DURATION_S * RATE_HZ)

        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _odom_cb(self, msg):
        self.odom = msg

    def _accel_cb(self, msg):
        self.accel = msg.accel.accel.linear.x
        self.peak = min(self.peak, self.accel)

    def _brake(self):
        if self.cli.wait_for_service(timeout_sec=2.0):
            self.cli.call_async(SetEmergency.Request(emergency=True))
            self.get_logger().warn(">>> VESZFEK")
        else:
            self.get_logger().error("/api/autoware/set/emergency nem elerheto")

    def _tick(self):
        if self.odom is None:
            return

        if not self.braked:
            self.braked = True
            self._brake()
            return

        p = self.odom.pose.pose
        tw = self.odom.twist.twist
        speed = math.hypot(tw.linear.x, tw.linear.y)

        if self.event is None:
            if self.accel > DECEL_THRESHOLD:
                self.get_logger().info(f"varok... a={self.accel:.2f}",
                                       throttle_duration_sec=1.0)
                return
            q = p.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            lat, lon, alt = self.geo.to_wgs(p.position.x, p.position.y, p.position.z)
            self.event = (lat, lon, alt, (90 - math.degrees(yaw)) % 360, speed)
            self.detection_time = time.time()
            self.get_logger().warn(
                f"ESEMENY: {lat:.7f},{lon:.7f} v={speed:.1f} m/s a={self.accel:.2f}")

        lat, lon, alt, heading, v0 = self.event
        d = copy.deepcopy(self.template)
        m = d["management"]
        # originatingStationId-t a Vanetza-NAP tolti ki, itt nem irjuk felul.
        m["actionId"]["sequenceNumber"] = 0
        m["detectionTime"] = round(self.detection_time, 3)
        m["referenceTime"] = round(time.time(), 3)
        m["eventPosition"]["latitude"] = lat
        m["eventPosition"]["longitude"] = lon
        m["eventPosition"]["altitude"]["altitudeValue"] = round(alt, 2)
        m["validityDuration"] = int(math.ceil(DURATION_S))
        m["stationType"] = STATION_TYPE

        d["situation"]["informationQuality"] = info_quality(self.peak)
        d["situation"]["eventType"] = {"ccAndScc": {"dangerousSituation99": 1}}

        if self.sent == 0:
            print(json.dumps(d, indent=2))

        self.mqtt.publish(MQTT_TOPIC, json.dumps(d))
        self.sent += 1
        self.get_logger().info(
            f"DENM {self.sent}/{self.total} peak={self.peak:.2f} m/s^2")

        if self.sent >= self.total:
            raise SystemExit

    def destroy_node(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    n = EeblSender()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()