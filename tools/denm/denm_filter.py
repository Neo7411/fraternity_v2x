#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
eebl_monitor.py — EEBL DENM + TrackedObjects osszekapcsolasa

Csak akkor jelez, ha a fekezo jarmu az ego ELOTT van, a sajat savban.

A Vanetza-NAP kimeno formatuma be van csomagolva:
    { "stationID": 175, "fields": { "denm": { "management": {...} } } }
A kuldo azonositasa a gyoker "stationID" mezobol tortenik — az
actionId.originatingStationId a JSON sablonbol jon es nem megbizhato.
"""

import json
import math
import struct
import time
from collections import defaultdict, deque

import numpy as np
import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import paho.mqtt.client as mqtt
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from autoware_perception_msgs.msg import TrackedObjects

# ---------------- BEALLITASOK ----------------
MQTT_BROKER = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC_IN = "vanetza/out/denm"

MAP_ORIGIN = (47.5316, 21.6273, 0.0)
OBJECTS_TOPIC = "/perception/object_recognition/tracking/objects"

FRONT_OFFSET_M = 3.79
LANE_WIDTH_M = 3.5
ONCOMING_DEG = 120.0
MATCH_RADIUS_M = 15.0
EVENT_TIMEOUT_S = 5.0
DECEL_WINDOW_S = 0.6
PUBLISH_HZ = 10.0

DEBUG_RAW = True          # az elso 3 beerkezo DENM nyers kiirasa
# ---------------------------------------------

V2X_TAG = b"V2X"
DANGEROUS_SITUATION = 99


class Geo:
    def __init__(self, lat0, lon0, h0):
        self.lla2ecef = pyproj.Transformer.from_crs(4979, 4978, always_xy=True)
        self.ecef2lla = pyproj.Transformer.from_crs(4978, 4979, always_xy=True)
        self.o = np.array(self.lla2ecef.transform(lon0, lat0, h0))
        la, lo = math.radians(lat0), math.radians(lon0)
        sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
        self.R = np.array([[-so, -sl * co, cl * co],
                           [co, -sl * so, cl * so],
                           [0.0, cl, sl]])

    def to_wgs(self, e, n, u=0.0):
        x, y, z = self.o + self.R @ np.array([e, n, u])
        lon, lat, h = self.ecef2lla.transform(x, y, z)
        return lat, lon, h

    def to_enu(self, lat, lon, alt=0.0):
        ecef = np.array(self.lla2ecef.transform(lon, lat, alt))
        return self.R.T @ (ecef - self.o)


def uuid_to_station_id(u):
    return struct.unpack(">I", bytes(u.uuid[:4]))[0]


def is_v2x_uuid(u):
    return bytes(u.uuid[4:7]) == V2X_TAG


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def kv(k, v):
    return KeyValue(key=str(k), value=str(v))


class DecelEstimator:
    def __init__(self, window_s=DECEL_WINDOW_S):
        self.window = window_s
        self.hist = defaultdict(lambda: deque(maxlen=60))

    def update(self, oid, t, speed):
        self.hist[oid].append((t, speed))

    def estimate(self, oid, t_now):
        pts = [(t, v) for t, v in self.hist.get(oid, ())
               if t_now - t <= self.window]
        if len(pts) < 3:
            return None
        ts = np.array([p[0] for p in pts])
        vs = np.array([p[1] for p in pts])
        if ts[-1] - ts[0] < 0.15:
            return None
        return float(np.polyfit(ts - ts[0], vs, 1)[0])

    def prune(self, alive):
        for k in list(self.hist):
            if k not in alive:
                del self.hist[k]


def kinematics_accel(obj):
    acc = getattr(obj.kinematics, "acceleration_with_covariance", None)
    if acc is None:
        return None
    ax, ay = acc.accel.linear.x, acc.accel.linear.y
    if abs(ax) < 1e-6 and abs(ay) < 1e-6:
        return None
    return float(ax)


class EeblMonitor(Node):

    def __init__(self):
        super().__init__("eebl_monitor")

        self.geo = Geo(*MAP_ORIGIN)
        self.ego = None
        self.objects = []
        self.decel = DecelEstimator()
        self.events = {}
        self.denm_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self._on_ego, 10)
        self.create_subscription(TrackedObjects, OBJECTS_TOPIC,
                                 self._on_objects, qos)

        self.pub_status = self.create_publisher(
            DiagnosticArray, "/v2x/eebl/status", 10)
        self.pub_objects = self.create_publisher(
            TrackedObjects, "/v2x/eebl/objects", 10)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_disconnect = self._on_disconnect
        self.mqtt.on_message = self._on_denm
        try:
            self.mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect hiba: {e}")

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.create_timer(5.0, self._heartbeat)

    # ------------------------------------------------ MQTT

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """A SUBSCRIBE csak a CONNACK utan mehet ki, kulonben a broker eldobja."""
        self.get_logger().info(
            f"MQTT csatlakozva (rc={rc}), feliratkozas: {MQTT_TOPIC_IN}")
        client.subscribe(MQTT_TOPIC_IN)

    def _on_disconnect(self, client, userdata, flags, rc, properties=None):
        self.get_logger().error(f"MQTT szetkapcsolt (rc={rc})")

    def _on_denm(self, client, userdata, msg):
        self.denm_count += 1
        try:
            raw = json.loads(msg.payload)
        except Exception as e:
            self.get_logger().error(f"JSON parse hiba: {e}")
            return

        if DEBUG_RAW and self.denm_count <= 3:
            self.get_logger().info(
                f"NYERS DENM #{self.denm_count}:\n{json.dumps(raw, indent=2)}")

        # A hasznos tartalom a fields.denm alatt van.
        denm = raw.get("fields", {}).get("denm") or raw
        try:
            sit = denm["situation"]
            cc = sit["eventType"]["ccAndScc"]
        except KeyError:
            self.get_logger().warn(
                f"Nincs situation/ccAndScc. Elerheto kulcsok: {list(denm)}")
            return

        key = next((k for k in cc if k.endswith(str(DANGEROUS_SITUATION))), None)
        if key is None:
            self.get_logger().info(f"Nem EEBL DENM, causeCode: {list(cc)}")
            return

        try:
            mgmt = denm["management"]
            pos = mgmt["eventPosition"]
        except KeyError as e:
            self.get_logger().warn(f"Hianyzo mezo: {e}")
            return

        # A kuldo valos azonositoja a gyoker stationID.
        sid = raw.get("stationID")
        if sid is None:
            sid = raw.get("fields", {}).get("header", {}).get("stationId")
        if sid is None:
            sid = mgmt.get("actionId", {}).get("originatingStationId")

        self.events[sid] = {
            "sid": sid,
            "subcause": cc[key],
            "info_quality": sit.get("informationQuality", 0),
            "lat": pos["latitude"],
            "lon": pos["longitude"],
            "rx_time": time.time(),
        }
        self.get_logger().warn(
            f"EEBL DENM: station {sid}, subcause {cc[key]}, "
            f"IQ {sit.get('informationQuality')}, "
            f"pos {pos['latitude']:.7f},{pos['longitude']:.7f}")

    # ------------------------------------------------ ROS bemenetek

    def _on_ego(self, msg):
        p = msg.pose.pose
        self.ego = (p.position.x, p.position.y, quat_to_yaw(p.orientation))

    def _on_objects(self, msg):
        self.objects = msg.objects
        now = time.time()
        alive = set()
        for o in msg.objects:
            oid = uuid_to_station_id(o.object_id)
            alive.add(oid)
            self.decel.update(
                oid, now, o.kinematics.twist_with_covariance.twist.linear.x)
        self.decel.prune(alive)

    def _heartbeat(self):
        self.get_logger().info(
            f"[status] DENM: {self.denm_count} | aktiv esemeny: "
            f"{len(self.events)} | objektum: {len(self.objects)} | "
            f"ego: {'ok' if self.ego else 'NINCS'}")

    # ------------------------------------------------ logika

    def _analyze(self, obj):
        ex, ey, eyaw = self.ego
        p = obj.kinematics.pose_with_covariance.pose
        dx, dy = p.position.x - ex, p.position.y - ey
        forward = dx * math.cos(eyaw) + dy * math.sin(eyaw)
        lateral = -dx * math.sin(eyaw) + dy * math.cos(eyaw)
        gap = forward - FRONT_OFFSET_M
        hdiff = norm_angle_deg(math.degrees(quat_to_yaw(p.orientation) - eyaw))

        if abs(hdiff) > ONCOMING_DEG:
            cat = "szembejovo"
        elif abs(lateral) < LANE_WIDTH_M / 2.0:
            cat = "sajat sav"
        elif abs(lateral) < LANE_WIDTH_M * 1.5:
            cat = "szomszed sav"
        else:
            cat = "tavoli sav"
        return gap, lateral, hdiff, cat

    def _match(self, event):
        for o in self.objects:
            if uuid_to_station_id(o.object_id) == event["sid"]:
                return o, "station_id"
        e, n, _ = self.geo.to_enu(event["lat"], event["lon"])
        best, best_d = None, MATCH_RADIUS_M
        for o in self.objects:
            p = o.kinematics.pose_with_covariance.pose.position
            d = math.hypot(p.x - e, p.y - n)
            if d < best_d:
                best, best_d = o, d
        return (best, f"pozicio_{best_d:.1f}m") if best else (None, None)

    def _tick(self):
        now = time.time()
        for sid in [s for s, e in self.events.items()
                    if now - e["rx_time"] > EVENT_TIMEOUT_S]:
            self.get_logger().info(f"EEBL esemeny lejart: station {sid}")
            del self.events[sid]

        if self.ego is None or not self.events:
            return

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        out = TrackedObjects()
        out.header = arr.header

        for event in self.events.values():
            obj, how = self._match(event)
            if obj is None:
                self.get_logger().warn(
                    f"station {event['sid']}: EEBL DENM van, tracked object nincs",
                    throttle_duration_sec=2.0)
                continue

            gap, lateral, hdiff, cat = self._analyze(obj)

            # CSAK ha elottunk van, a sajat savunkban
            if gap <= 0.0 or cat != "sajat sav":
                self.get_logger().info(
                    f"station {event['sid']}: figyelmen kivul "
                    f"(gap={gap:.1f} m, {cat})", throttle_duration_sec=2.0)
                continue

            oid = uuid_to_station_id(obj.object_id)
            a_field = kinematics_accel(obj)
            a_diff = self.decel.estimate(oid, now)
            if a_field is not None:
                decel, src = a_field, "kinematics"
            elif a_diff is not None:
                decel, src = a_diff, "derivalt"
            else:
                decel, src = float("nan"), "nincs"

            p = obj.kinematics.pose_with_covariance.pose.position
            lat, lon, _ = self.geo.to_wgs(p.x, p.y, p.z)
            speed = obj.kinematics.twist_with_covariance.twist.linear.x

            arr.status.append(DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                name=f"eebl/station_{event['sid']}",
                message=f"EEBL elottunk: {gap:.1f} m",
                values=[
                    kv("station_id", event["sid"]),
                    kv("matched_by", how),
                    kv("subcause", event["subcause"]),
                    kv("info_quality", event["info_quality"]),
                    kv("gap_m", f"{gap:.2f}"),
                    kv("lateral_m", f"{lateral:.2f}"),
                    kv("heading_diff_deg", f"{hdiff:.1f}"),
                    kv("speed_mps", f"{speed:.2f}"),
                    kv("decel_mps2", f"{decel:.2f}"),
                    kv("decel_source", src),
                    kv("obj_lat", f"{lat:.7f}"),
                    kv("obj_lon", f"{lon:.7f}"),
                    kv("event_lat", f"{event['lat']:.7f}"),
                    kv("event_lon", f"{event['lon']:.7f}"),
                    kv("is_v2x", is_v2x_uuid(obj.object_id)),
                ]))
            out.objects.append(obj)

            self.get_logger().warn(
                f">>> EEBL station {event['sid']}: {gap:.1f} m elottunk, "
                f"{speed * 3.6:.1f} km/h, lassulas {decel:.2f} m/s^2 ({src})")

        if arr.status:
            self.pub_status.publish(arr)
            self.pub_objects.publish(out)

    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    n = EeblMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()