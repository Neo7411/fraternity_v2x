#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kiirja az ego elott levo TrackedObject-eket a station ID-jukkal, sav szerint
kategorizalva.

Kategorizalas (egyenes uton ervenyes, kanyarban nem):
  - oldaliranyu eltolas |lateral| < lane_width/2      -> sajat sav
  - ennel nagyobb, de < 1.5 * lane_width              -> szomszed sav
  - irany-kulonbseg > oncoming_threshold_deg          -> szembejovo
"""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrackedObjects

V2X_TAG = b'V2X'


def uuid_to_station_id(u):
    return struct.unpack('>I', bytes(u.uuid[:4]))[0]


def is_v2x_uuid(u):
    return bytes(u.uuid[4:7]) == V2X_TAG


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle_deg(deg):
    """[-180, 180] tartomanyra normalas."""
    return (deg + 180.0) % 360.0 - 180.0


class FrontObjectsMonitor(Node):
    def __init__(self):
        super().__init__('front_objects_monitor')

        p = self.declare_parameter
        self.objects_topic = p(
            'objects_topic', '/perception/object_recognition/tracking/objects').value

        # base_link -> jarmu orra (wheelbase + front_overhang), Lexus RX450h: ~3.79 m
        self.front_offset = float(p('front_offset_m', 3.79).value)
        self.lane_width = float(p('lane_width_m', 3.5).value)
        self.oncoming_deg = float(p('oncoming_threshold_deg', 120.0).value)

        # Mit irjunk ki: csak a sajat sav, vagy a szomszed sav es a szembejovo is
        self.only_own_lane = bool(p('only_own_lane', True).value)
        self.show_oncoming = bool(p('show_oncoming', False).value)
        self.max_distance = float(p('max_distance_m', 0.0).value)  # 0 = nincs korlat
        self.print_period_s = float(p('print_period_s', 1.0).value)

        self.ego = None
        self.objects = []

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 self._on_ego, 10)
        self.create_subscription(TrackedObjects, self.objects_topic,
                                 self._on_objects, qos)

        self.create_timer(self.print_period_s, self._print_tick)
        self.get_logger().info(
            f"Figyelem: '{self.objects_topic}' | savszelesseg={self.lane_width:.1f} m, "
            f"csak sajat sav={self.only_own_lane}")

    def _on_ego(self, msg):
        pose = msg.pose.pose
        self.ego = (pose.position.x, pose.position.y, quat_to_yaw(pose.orientation))

    def _on_objects(self, msg):
        self.objects = msg.objects

    def _analyze(self, obj):
        """(gap, lateral, heading_diff, kategoria) az ego base_link rendszereben."""
        ego_x, ego_y, ego_yaw = self.ego
        pose = obj.kinematics.pose_with_covariance.pose
        dx = pose.position.x - ego_x
        dy = pose.position.y - ego_y

        forward = dx * math.cos(ego_yaw) + dy * math.sin(ego_yaw)
        lateral = -dx * math.sin(ego_yaw) + dy * math.cos(ego_yaw)
        gap = forward - self.front_offset

        obj_yaw = quat_to_yaw(pose.orientation)
        heading_diff = norm_angle_deg(math.degrees(obj_yaw - ego_yaw))

        if abs(heading_diff) > self.oncoming_deg:
            category = 'szembejovo'
        elif abs(lateral) < self.lane_width / 2.0:
            category = 'sajat sav'
        elif abs(lateral) < self.lane_width * 1.5:
            category = 'szomszed sav'
        else:
            category = 'tavoli sav'

        return gap, lateral, heading_diff, category

    def _print_tick(self):
        if self.ego is None:
            self.get_logger().warn('Nincs ego pozicio (/localization/kinematic_state)',
                                   throttle_duration_sec=5.0)
            return

        rows = []
        for obj in self.objects:
            gap, lateral, heading_diff, category = self._analyze(obj)

            if gap <= 0.0:
                continue
            if self.max_distance > 0.0 and gap > self.max_distance:
                continue
            if category == 'szembejovo' and not self.show_oncoming:
                continue
            if self.only_own_lane and category != 'sajat sav':
                continue

            rows.append((
                gap, uuid_to_station_id(obj.object_id), is_v2x_uuid(obj.object_id),
                lateral, heading_diff, category,
                obj.kinematics.twist_with_covariance.twist.linear.x))

        if not rows:
            self.get_logger().info('Nincs objektum az ego elott')
            return

        rows.sort(key=lambda r: r[0])
        lines = [f'--- {len(rows)} objektum az ego elott ---']
        for gap, sid, v2x, lateral, hdiff, category, speed in rows:
            side = 'bal' if lateral > 0 else 'jobb'
            src = 'V2X' if v2x else 'lokalis'
            lines.append(
                f'  ID {sid:<10} {gap:7.1f} m elore  '
                f'{abs(lateral):5.1f} m {side:<4}  '
                f'{speed * 3.6:6.1f} km/h  '
                f'd_irany={hdiff:+6.1f} deg  [{category}] [{src}]')
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = FrontObjectsMonitor()
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