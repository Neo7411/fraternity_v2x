#!/usr/bin/env python3
"""EEBL DENM kuldo.

Megallitja az EGO-t (ROS service), es a lassulas pillanataban a terkep
projekciojaval GPS-re szamolt sajat poziciot DENM-ben kikuldi MQTT-n.

A pozicio ugyanazzal a projekcioval megy, amit a terkep
map_projector_info.yaml-ja megad -- ez KRITIKUS: a vevo oldal
(denm_receive.py, cam_receiver.py) is ezzel szamol vissza, es ha a ketto
nem egyezik, az esemeny tobb szaz kilometerre kerul a valodi helyetol.
"""
import copy
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import paho.mqtt.client as mqtt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import AccelWithCovarianceStamped
from tier4_external_api_msgs.srv import SetEmergency

# A fraternity_v2x telepitett ROS csomag (source /home/aw/dev/install/setup.bash).
from fraternity_v2x.tools.map_tools import (
    MapProjection, enu_yaw_to_heading_deg, quat_to_yaw,
)


def info_quality(decel):
    """Lassulas -> informationQuality (0..7). Erosebb fek = megbizhatobb."""
    a = abs(decel)
    for lim, q in ((1, 1), (2, 2), (3, 3), (4.5, 4), (6, 5), (8, 6)):
        if a < lim:
            return q
    return 7


class DenmSender(Node):

    def __init__(self):
        super().__init__('denm_sender')

        p = self.declare_parameter

        self.broker = p('mqtt_broker', '127.0.0.1').value
        self.port = int(p('mqtt_port', 1883).value)
        self.topic = p('mqtt_topic', 'vanetza/in/denm').value
        self.template_path = p('template', '/vanetza/examples/in_denm.json').value

        self.map_path = p('map_path', '/home/aw/maps/highway2').value

        self.rate_hz = float(p('rate_hz', 10.0).value)
        self.duration_s = float(p('duration_s', 2.0).value)
        self.station_type = int(p('station_type', 5).value)

        # Ennel erosebb lassulasnal indul az esemeny.
        self.decel_threshold = float(p('decel_threshold', -3.0).value)

        # Ne varjunk lassulasra: azonnal kuldjunk a jelenlegi allapottal.
        self.force = bool(p('force', False).value)

        # Indulaskor keruljon-e veszfek az EGO-ra (a ROS service call).
        self.auto_brake = bool(p('auto_brake', True).value)
        self.brake_srv = p('brake_service', '/api/autoware/set/emergency').value

        # A LocationContainer eventSpeed/heading mezoit nem minden Vanetza
        # sema fogadja el -- ha elszall a kodolas, kapcsold ki.
        self.with_location = bool(p('with_location_container', True).value)

        with open(self.template_path) as f:
            self.template = json.load(f)

        self.geo = MapProjection(self.map_path, self.get_logger())

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.odom = None
        self.accel = 0.0
        self.peak = 0.0
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 self._on_odom, qos)
        self.create_subscription(AccelWithCovarianceStamped,
                                 '/localization/acceleration', self._on_accel, qos)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.connect(self.broker, self.port, 60)
        self.mqtt.loop_start()

        self.cli = self.create_client(SetEmergency, self.brake_srv)
        self.braked = False
        self.event = None
        self.seq = int(p('sequence_number', 0).value)
        self.sent = 0
        self.total = int(self.duration_s * self.rate_hz)

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f'DENM -> {self.topic} | {self.rate_hz:.0f} Hz, {self.duration_s:.1f} s')

    # ------------------------------------------------------------ bemenet

    def _on_odom(self, msg):
        self.odom = msg

    def _on_accel(self, msg):
        self.accel = msg.accel.accel.linear.x
        self.peak = min(self.peak, self.accel)

    def _brake(self):
        """Veszfek az Autoware-nek. Ettol all meg az auto."""
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'{self.brake_srv} nem elerheto')
            return
        self.cli.call_async(SetEmergency.Request(emergency=True))
        self.get_logger().warn('>>> VESZFEK kerese elkuldve')

    # ------------------------------------------------------------ kimenet

    def _build(self):
        lat, lon, alt, heading, speed = self.event

        d = copy.deepcopy(self.template)
        m = d['management']
        # Az originatingStationId a sablonbol orokolt ertek marad -- a NAP NEM
        # irja felul (lasd out_denm.json). Az ado valodi azonositoja a burkolo
        # `stationID`, a vevo is azt parositja.
        m['actionId']['sequenceNumber'] = self.seq
        m['detectionTime'] = round(self.detection_time, 3)
        m['referenceTime'] = round(time.time(), 3)
        m['eventPosition']['latitude'] = lat
        m['eventPosition']['longitude'] = lon
        m['eventPosition']['altitude']['altitudeValue'] = round(alt, 2)
        m['validityDuration'] = int(math.ceil(self.duration_s))
        m['stationType'] = self.station_type

        # force modban nem mertunk lassulast, ne latszodjon gyenge minosegunek.
        quality = 7 if (self.force and self.peak == 0.0) \
            else info_quality(self.peak)

        d['situation'] = {
            'informationQuality': quality,
            # EEBL: veszfekezo jarmu.
            'eventType': {'ccAndScc': {'dangerousSituation99': 1}},
        }

        if self.with_location:
            # Itt megy ki a fekezesi sebesseg es az irany.
            d['location'] = {
                'eventSpeed': {
                    'speedValue': round(speed, 2),
                    'speedConfidence': 127,
                },
                'eventPositionHeading': {
                    'headingValue': round(heading, 1),
                    'headingConfidence': 127,
                },
                'traces': [],
            }
        return d

    def _tick(self):
        if self.odom is None:
            self.get_logger().warn('nincs /localization/kinematic_state',
                                   throttle_duration_sec=2.0)
            return

        if self.auto_brake and not self.braked:
            self.braked = True
            self._brake()
            return

        # Az esemenyt a lassulas pillanata rogziti; utana mar ez a pozicio megy.
        if self.event is None:
            if not self.force and self.accel > self.decel_threshold:
                self.get_logger().info(
                    f'varok a fekezesre... a={self.accel:.2f} m/s^2 '
                    f'(kuszob {self.decel_threshold}) -- force:=true megkeruli',
                    throttle_duration_sec=2.0)
                return

            pp = self.odom.pose.pose
            tw = self.odom.twist.twist
            yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y,
                              pp.orientation.z, pp.orientation.w)
            lat, lon, alt = self.geo.to_wgs84(
                pp.position.x, pp.position.y, pp.position.z)
            self.event = (lat, lon, alt, enu_yaw_to_heading_deg(yaw),
                          math.hypot(tw.linear.x, tw.linear.y))
            self.detection_time = time.time()
            self.get_logger().warn(
                f'ESEMENY: map({pp.position.x:.1f}, {pp.position.y:.1f}) -> '
                f'{lat:.7f},{lon:.7f} | v={self.event[4]:.1f} m/s '
                f'a={self.accel:.2f} m/s^2')

        d = self._build()
        if self.sent == 0:
            self.get_logger().info(json.dumps(d, indent=2))

        self.mqtt.publish(self.topic, json.dumps(d))
        self.sent += 1
        self.get_logger().info(
            f'DENM {self.sent}/{self.total} peak={self.peak:.2f} m/s^2')

        if self.sent >= self.total:
            raise SystemExit

    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    try:
        node = DenmSender()
    except Exception:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
