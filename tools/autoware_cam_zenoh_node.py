#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autoware_cam_zenoh_node.py

Autoware ego pozicio -> ETSI CAM JSON -> vanetza-nap (ZENOH, key: vanetza/in/cam)

Ugyanaz mint az MQTT-s valtozat, csak a transport Zenoh:
  - zenoh kliens csatlakozik a socktap router-ehez (tcp/127.0.0.1:7447)
  - a CAM JSON a 'vanetza/in/cam' Zenoh kulcsra megy ki (put)

A pozicio forrasa valaszthato:
  - kinematic_state : /localization/kinematic_state (nav_msgs/Odometry)  [ajanlott]
  - tf              : map -> base_link TF lookup

A JSON sema az in_cam.json peldat tukrozi. A VALOS mezok decimalis SI egysegben
mennek (fok, m/s, deg/s, m/s^2) - a Vanetza skalazza ETSI fixpontos formara.

Fuggosegek:  rclpy, tf2_ros, numpy, pyproj, eclipse-zenoh
    pip install pyproj eclipse-zenoh
"""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

import zenoh

# ITS epoch = 2004-01-01T00:00:00 UTC, UNIX ms
ITS_EPOCH_MS = 1072915200 * 1000


# ---------------------------------------------------------------------------
# ENU (map frame) -> WGS84  (origin = a terkep origo lat/lon/alt-ja)
# ---------------------------------------------------------------------------
class EnuToWgs84:
    """Lokalis ENU (x=East, y=North, z=Up) -> geodetic, ECEF rotacioval. pyproj alapu."""

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
    """ENU yaw (CCW Eastol) -> ETSI heading (fok, CW Eszaktol, 0..360)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class AutowareCamZenoh(Node):
    def __init__(self):
        super().__init__('autoware_cam_zenoh')

        # ---- parameterek ----
        p = self.declare_parameter
        # a socktap router endpointja (zenoh_local_only=true -> 127.0.0.1:7447)
        self.zenoh_endpoint = p('zenoh_endpoint', 'tcp/127.0.0.1:7447').value
        self.zenoh_mode = p('zenoh_mode', 'peer').value      # client vagy peer
        self.key_in = p('zenoh_key_in', 'vanetza/in/cam').value

        self.station_id = int(p('station_id', 1).value)
        self.station_type = int(p('station_type', 5).value)    # 5 = passengerCar
        self.veh_len = float(p('vehicle_length_m', 0.0).value)  # 0 -> unavailable
        self.veh_wid = float(p('vehicle_width_m', 0.0).value)   # 0 -> unavailable

        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_link').value
        self.pose_source = p('pose_source', 'kinematic_state').value  # vagy 'tf'
        self.rate_hz = float(p('rate_hz', 10.0).value)

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = EnuToWgs84(lat0, lon0, alt0)
        self.get_logger().info(f'Map origin: lat={lat0} lon={lon0} alt={alt0}')

        # ---- ROS IO ----
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.last_odom = None
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 self._on_odom, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._prev = None  # finite-diff fallback

        # ---- ZENOH ----
        conf = zenoh.Config()
        conf.insert_json5('mode', f'"{self.zenoh_mode}"')
        conf.insert_json5('connect/endpoints', json.dumps([self.zenoh_endpoint]))
        self.zsession = zenoh.open(conf)
        self.pub = self.zsession.declare_publisher(self.key_in)
        self.get_logger().info(
            f'Zenoh {self.zenoh_mode} -> {self.zenoh_endpoint} key={self.key_in} '
            f'source={self.pose_source} rate={self.rate_hz}Hz')

        self.create_timer(1.0 / self.rate_hz, self._tick)

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _get_state(self):
        """visszaad: (e, n, u, yaw_rad, vx, vy, yaw_rate) vagy None"""
        if self.pose_source == 'tf':
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
            except (LookupException, ExtrapolationException):
                return None
            t = tf.transform.translation
            q = tf.transform.rotation
            e, n, u = t.x, t.y, t.z
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        else:
            if self.last_odom is None:
                return None
            pp = self.last_odom.pose.pose
            e, n, u = pp.position.x, pp.position.y, pp.position.z
            yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y,
                              pp.orientation.z, pp.orientation.w)

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
                    ground = math.hypot(de, dn) / dt
                    dh = (heading - self._prev[3] + 540) % 360 - 180
                    yaw_rate = math.radians(dh) / dt
                    vx, vy = ground, 0.0
            self._prev = (now, e, n, heading)
            if vx is None:
                vx, vy, yaw_rate = 0.0, 0.0, 0.0

        return e, n, u, yaw, vx, vy, yaw_rate

    def _gen_delta_time(self):
        return int((time.time() * 1000 - ITS_EPOCH_MS)) % 65536

    def _build_cam(self, lat, lon, heading_deg, speed_mps, drive_dir, yaw_rate_dps):
        veh_len = round(self.veh_len, 1) if self.veh_len > 0 else 1023
        veh_wid = round(self.veh_wid, 1) if self.veh_wid > 0 else 62
        return {
            "camParameters": {
                "basicContainer": {
                    "stationType": self.station_type,
                    "referencePosition": {
                        "latitude": round(lat, 7),
                        "longitude": round(lon, 7),
                        "positionConfidenceEllipse": {
                            "semiMajorAxisLength": 4095,
                            "semiMinorAxisLength": 4095,
                            "semiMajorAxisOrientation": 3601
                        },
                        "altitude": {"altitudeValue": 800001, "altitudeConfidence": 15}
                    }
                },
                "highFrequencyContainer": {
                    "basicVehicleContainerHighFrequency": {
                        "heading": {"headingValue": round(heading_deg, 1),
                                    "headingConfidence": 127},
                        "speed": {"speedValue": round(max(0.0, speed_mps), 2),
                                  "speedConfidence": 127},
                        "driveDirection": drive_dir,
                        "vehicleLength": {"vehicleLengthValue": veh_len,
                                          "vehicleLengthConfidenceIndication": 4},
                        "vehicleWidth": veh_wid,
                        "longitudinalAcceleration": {"value": 0.0, "confidence": 102},
                        "curvature": {"curvatureValue": 1023, "curvatureConfidence": 7},
                        "curvatureCalculationMode": 2,
                        "yawRate": {"yawRateValue": round(yaw_rate_dps, 2),
                                    "yawRateConfidence": 8},
                        "accelerationControl": {
                            "brakePedalEngaged": False, "gasPedalEngaged": False,
                            "emergencyBrakeEngaged": False, "collisionWarningEngaged": False,
                            "accEngaged": False, "cruiseControlEngaged": False,
                            "speedLimiterEngaged": False
                        },
                        "steeringWheelAngle": {"steeringWheelAngleValue": 512,
                                               "steeringWheelAngleConfidence": 127}
                    }
                },
                "lowFrequencyContainer": {
                    "basicVehicleContainerLowFrequency": {
                        "vehicleRole": 0,
                        "exteriorLights": {
                            "lowBeamHeadlightsOn": False, "highBeamHeadlightsOn": False,
                            "leftTurnSignalOn": False, "rightTurnSignalOn": False,
                            "daytimeRunningLightsOn": False, "reverseLightOn": False,
                            "fogLightOn": False, "parkingLightsOn": False
                        },
                        "pathHistory": []
                    }
                }
            },
            "generationDeltaTime": self._gen_delta_time()
        }

    def _tick(self):
        st = self._get_state()
        if st is None:
            self.get_logger().warn('Nincs meg pozicio (odom/TF) ...', throttle_duration_sec=2.0)
            return
        e, n, u, yaw, vx, vy, yaw_rate = st
        lat, lon, _ = self.geo.convert(e, n, u)
        heading_deg = enu_yaw_to_heading_deg(yaw)
        speed = math.hypot(vx, vy)
        drive_dir = 0 if vx >= 0 else 1
        yaw_rate_dps = math.degrees(yaw_rate)

        cam = self._build_cam(lat, lon, heading_deg, speed, drive_dir, yaw_rate_dps)
        self.pub.put(json.dumps(cam))

    def destroy_node(self):
        try:
            self.zsession.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = AutowareCamZenoh()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
