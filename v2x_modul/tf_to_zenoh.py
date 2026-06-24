#!/usr/bin/env python3
"""
tf_to_cam_zenoh.py
Autoware /tf (map -> base_link) -> CAM JSON -> Zenoh put -> vanetza/in/cam

Függőségek:
    pip install eclipse-zenoh pymap3d
    (rclpy, tf2_ros: ROS 2 Humble környezetből)

Indítás:
    ros2 run <pkg> tf_to_cam_zenoh.py   # vagy: python3 tf_to_cam_zenoh.py
    paraméterekkel:
    python3 tf_to_cam_zenoh.py --ros-args \
        -p map_origin_lat:=47.53 -p map_origin_lon:=21.62 -p map_origin_alt:=120.0 \
        -p station_id:=1 -p zenoh_endpoint:=tcp/127.0.0.1:7447
"""
import json
import math
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from tf2_ros import TransformException

import pymap3d as pm
import zenoh

# ETSI ITS epoch: 2004-01-01T00:00:00 UTC (generationDeltaTime alapja)
ITS_EPOCH_MS = int(datetime(2004, 1, 1, tzinfo=timezone.utc).timestamp() * 1000)


def quat_to_yaw(x, y, z, w):
    """Yaw (rad) a map/ENU síkban, CCW az East (+x) tengelytől."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def enu_yaw_to_heading(yaw_rad):
    """ENU yaw (CCW East-től) -> ETSI heading (CW North-tól), fok 0..360."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


class TfToCamZenoh(Node):
    def __init__(self):
        super().__init__("tf_to_cam_zenoh")

        # --- paraméterek ---
        self.declare_parameter("map_origin_lat", 47.53)
        self.declare_parameter("map_origin_lon", 21.62)
        self.declare_parameter("map_origin_alt", 120.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("station_id", 1)
        self.declare_parameter("station_type", 5)        # 5 = passengerCar
        self.declare_parameter("vehicle_length", 4.9)    # m (RX450h ~)
        self.declare_parameter("vehicle_width", 1.9)     # m
        self.declare_parameter("rate_hz", 10.0)          # CAM 1..10 Hz
        self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447")
        self.declare_parameter("cam_topic", "vanetza/in/cam")

        gp = self.get_parameter
        self.lat0 = gp("map_origin_lat").value
        self.lon0 = gp("map_origin_lon").value
        self.alt0 = gp("map_origin_alt").value
        self.map_frame = gp("map_frame").value
        self.base_frame = gp("base_frame").value
        self.station_id = int(gp("station_id").value)
        self.station_type = int(gp("station_type").value)
        self.veh_len = float(gp("vehicle_length").value)
        self.veh_wid = float(gp("vehicle_width").value)
        rate = float(gp("rate_hz").value)
        endpoint = gp("zenoh_endpoint").value
        self.cam_topic = gp("cam_topic").value

        # --- tf2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Zenoh client session a Vanetza routeréhez ---
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", json.dumps([endpoint]))
        self.zsession = zenoh.open(conf)
        self.pub = self.zsession.declare_publisher(self.cam_topic)
        self.get_logger().info(f"Zenoh -> {endpoint}, kulcs: {self.cam_topic}")

        # sebességhez: előző pozíció + idő
        self._last_e = None
        self._last_n = None
        self._last_t = None

        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"tf lookup nem sikerült: {ex}", throttle_duration_sec=2.0)
            return

        t = tf.transform.translation       # ENU (kelet=x, észak=y, fel=z) a map-ben
        q = tf.transform.rotation
        east, north, up = t.x, t.y, t.z

        # ENU -> WGS84 a map origó alapján
        lat, lon, alt = pm.enu2geodetic(east, north, up, self.lat0, self.lon0, self.alt0)

        # heading az orientációból
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        heading = enu_yaw_to_heading(yaw)

        # sebesség véges differenciából
        stamp = tf.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        speed = 0.0
        if self._last_t is not None:
            dt = now - self._last_t
            if dt > 1e-3:
                speed = math.hypot(east - self._last_e, north - self._last_n) / dt
        self._last_e, self._last_n, self._last_t = east, north, now

        cam = self.build_cam(lat, lon, heading, speed)
        self.pub.put(json.dumps(cam))
        self.get_logger().info(
            f"CAM tx: lat={lat:.6f} lon={lon:.6f} hdg={heading:.1f} v={speed:.2f}",
            throttle_duration_sec=1.0,
        )

    def build_cam(self, lat, lon, heading, speed):
        gen_delta = int(self._unix_ms() - ITS_EPOCH_MS) % 65536
        return {
            "stationId": self.station_id,
            "generationDeltaTime": gen_delta,
            "camParameters": {
                "basicContainer": {
                    "stationType": self.station_type,
                    "referencePosition": {
                        "latitude": lat,
                        "longitude": lon,
                        "positionConfidenceEllipse": {
                            "semiMajorAxisLength": 4095,
                            "semiMinorAxisLength": 4095,
                            "semiMajorAxisOrientation": 3601,
                        },
                        "altitude": {"altitudeValue": 800001, "altitudeConfidence": 15},
                    },
                },
                "highFrequencyContainer": {
                    "basicVehicleContainerHighFrequency": {
                        "heading": {"headingValue": round(heading, 1), "headingConfidence": 127},
                        "speed": {"speedValue": round(speed, 2), "speedConfidence": 127},
                        "driveDirection": 0,  # 0=forward
                        "vehicleLength": {
                            "vehicleLengthValue": self.veh_len,
                            "vehicleLengthConfidenceIndication": 0,
                        },
                        "vehicleWidth": self.veh_wid,
                        "longitudinalAcceleration": {"value": 0.0, "confidence": 102},
                        "curvature": {"curvatureValue": 1023, "curvatureConfidence": 7},
                        "curvatureCalculationMode": 2,
                        "yawRate": {"yawRateValue": 0.0, "yawRateConfidence": 8},
                        "accelerationControl": {
                            "brakePedalEngaged": False, "gasPedalEngaged": False,
                            "emergencyBrakeEngaged": False, "collisionWarningEngaged": False,
                            "accEngaged": False, "cruiseControlEngaged": False,
                            "speedLimiterEngaged": False,
                        },
                        "steeringWheelAngle": {
                            "steeringWheelAngleValue": 512,
                            "steeringWheelAngleConfidence": 127,
                        },
                    }
                },
            },
        }

    @staticmethod
    def _unix_ms():
        return datetime.now(timezone.utc).timestamp() * 1000.0

    def destroy_node(self):
        try:
            self.zsession.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TfToCamZenoh()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()