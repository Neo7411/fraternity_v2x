#!/usr/bin/env python3
"""EEBL DENM vevo.

MQTT-n DENM-et fogad, es ha az ado jarmu az EGO sajat savjaban, elottunk
van, kiirja az ID-jat, a GPS-et es a fekezesi sebesseget.

Az "elottem van-e" dontes a tracking topicrol jon (a CAM receiver teszi oda
a V2X jarmuveket), lanelet2 Frenet-koordinatakban. A DENM-et a burkolo
stationID parositja az ott latott objektumhoz. Ha az ado nem latszik a
trackingben (nem kuld CAM-et), a DENM sajat eventPosition-jere esunk vissza.
"""
import json
import math
import struct
import threading

import rclpy
from rclpy.node import Node

import lanelet2
from lanelet2.core import BasicPoint2d, LaneletSequence
from lanelet2.geometry import findNearest, to2D, toArcCoordinates

import paho.mqtt.client as mqtt

from nav_msgs.msg import Odometry
from autoware_perception_msgs.msg import TrackedObjects

# A fraternity_v2x telepitett ROS csomag (source /home/aw/dev/install/setup.bash).
# Ugyanaz a projekcio, amivel a denm_send.py a GPS-t szamolja.
from fraternity_v2x.tools.map_tools import MapProjection

MAP = '/home/aw/maps/highway2'
OBJ_TOPIC = '/perception/object_recognition/tracking/objects'
EGO_TOPIC = '/localization/kinematic_state'

LANE_W = 3.5        # savszelesseg a sav-offset kerekitesehez
HEADING_TOL = 90.0  # ennel nagyobb elteresnel ellenirany a sav


def obj_id(u):
    """CAM-bol jovo objektum UUID-ja: [stationID 4B BE][b'V2X'][9 x 0x00]."""
    if bytes(u.uuid[4:7]) == b'V2X':
        return str(struct.unpack('>I', bytes(u.uuid[:4]))[0])
    return bytes(u.uuid).hex()


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_denm(data):
    """(station_id, seq, lat, lon, speed) a DENM-bol. speed lehet None.

    Az ado azonositoja a burkolo `stationID` / `header.stationId` -- ezt tolti
    ki a Vanetza-NAP, es a CAM receiver is ezt teszi az objektum UUID-jaba.
    Az `originatingStationId` a sablonbol orokolt ertek marad, azt a NAP nem
    irja felul, tehat parositasra alkalmatlan.
    """
    f = data.get('fields', data)
    d = f.get('denm', f)

    m = d['management']
    sid = (data.get('stationID')
           or f.get('header', {}).get('stationId')
           or m['actionId'].get('originatingStationId'))
    seq = m['actionId'].get('sequenceNumber', 0)
    pos = m['eventPosition']

    # A fekezesi sebesseg a LocationContainer-ben jon (lasd denm_send.py).
    speed = None
    loc = d.get('location')
    if loc and isinstance(loc.get('eventSpeed'), dict):
        speed = float(loc['eventSpeed'].get('speedValue'))

    return (int(sid), int(seq),
            float(pos['latitude']), float(pos['longitude']), speed)


class DenmReceiver(Node):

    def __init__(self):
        super().__init__('denm_receiver')

        p = self.declare_parameter

        self.map_path = p('map_path', MAP).value
        self.broker = p('mqtt_broker', '127.0.0.1').value
        self.port = int(p('mqtt_port', 1883).value)
        self.topic = p('mqtt_denm_topic', 'vanetza/out/denm').value

        osm = self.map_path
        if not osm.endswith('.osm'):
            osm = osm.rstrip('/') + '/lanelet2_map.osm'
        self.geo = MapProjection(osm.rsplit('/', 1)[0], self.get_logger())
        # Ugyanaz a projektor, amit a map_projector_info.yaml megad
        # (highway2: MGRS) -- kulonben a savok nem egyeznenek az EGO-val.
        self.map = lanelet2.io.load(osm, self.geo.projector)
        self.graph = lanelet2.routing.RoutingGraph(
            self.map, lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle))

        self.ego = None
        # station_id -> tavolsag (m), csak az elottunk, sajat savban levok
        self.front = {}
        self._lock = threading.Lock()
        # (station_id, seq) parosokat egyszer jelentunk
        self._seen = set()
        self.rx = 0

        self.create_subscription(Odometry, EGO_TOPIC, self._on_ego, 10)
        self.create_subscription(TrackedObjects, OBJ_TOPIC, self._on_objs, 10)
        self.create_timer(5.0, self._heartbeat)

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_denm
        self.mqtt.connect(self.broker, self.port, 60)
        self.mqtt.loop_start()

        # A kiterjedes elarulja a rossz projekciot: ha 0..0, vagy a nagysagrend
        # nem stimmel az EGO poziciojahoz, nem a savdontes a hibas.
        xs = [pt.x for pt in self.map.pointLayer]
        ys = [pt.y for pt in self.map.pointLayer]
        self.get_logger().info(
            f'{len(self.map.laneletLayer)} lanelet | '
            f'x {min(xs):.1f}..{max(xs):.1f}  y {min(ys):.1f}..{max(ys):.1f} m | '
            f'DENM: {self.topic}')

    # ------------------------------------------------------------ elottem

    def _on_ego(self, msg):
        p = msg.pose.pose.position
        self.ego = (p.x, p.y, yaw_of(msg.pose.pose.orientation))

    def _ego_lanelet(self, x, y, yaw):
        """A legkozelebbi olyan lanelet, ami az EGO-val egy iranyba fut."""
        for _, ll in findNearest(self.map.laneletLayer, BasicPoint2d(x, y), 4):
            c = list(ll.centerline)
            lane_yaw = math.atan2(c[-1].y - c[0].y, c[-1].x - c[0].x)
            d = abs(math.degrees(yaw - lane_yaw) + 180.0) % 360.0 - 180.0
            if abs(d) <= HEADING_TOL:
                return ll
        return None

    def _reference(self):
        """Az EGO lanelet-jetol elore fuzott lanc kozepvonala + az EGO arc-ja."""
        if self.ego is None:
            return None
        x, y, yaw = self.ego
        ll = self._ego_lanelet(x, y, yaw)
        if ll is None:
            self.get_logger().warn('EGO nincs egyetlen savon sem',
                                   throttle_duration_sec=5.0)
            return None

        chain, cur, seen = [ll], ll, {ll.id}
        for _ in range(20):
            nxt = self.graph.following(cur)
            if not nxt or nxt[0].id in seen:
                break
            cur = nxt[0]
            seen.add(cur.id)
            chain.append(cur)

        # A centerline 3D, a toArcCoordinates 2D vonalat var.
        center = to2D(LaneletSequence(chain).centerline)
        return center, toArcCoordinates(center, BasicPoint2d(x, y))

    def _ahead(self, center, ego_arc, x, y):
        """Hany meterre van elottunk, sajat savban. None, ha nem ott van."""
        arc = toArcCoordinates(center, BasicPoint2d(x, y))
        ds = arc.length - ego_arc.length
        if ds <= 0.0:
            return None
        if math.floor((arc.distance - ego_arc.distance) / LANE_W + 0.5):
            return None
        return ds

    def _on_objs(self, msg):
        with self._lock:
            ref = self._reference()
            if ref is None:
                return
            center, ego_arc = ref

            front = {}
            for o in msg.objects:
                p = o.kinematics.pose_with_covariance.pose.position
                ds = self._ahead(center, ego_arc, p.x, p.y)
                if ds is not None:
                    front[obj_id(o.object_id)] = ds
            self.front = front

    def _heartbeat(self):
        """Csend eseten is latszodjon, hol tartunk."""
        if self.ego is None:
            self.get_logger().warn(
                f'[status] nincs EGO pozicio ({EGO_TOPIC}) | DENM: {self.rx}')
            return
        with self._lock:
            f = dict(self.front)
        self.get_logger().info(
            f'[status] DENM: {self.rx} | elottem: '
            + (', '.join(f'{i} ({d:.0f} m)' for i, d in sorted(
                f.items(), key=lambda kv: kv[1])) if f else '-'))

    # ------------------------------------------------------------ DENM

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        """A SUBSCRIBE csak a CONNACK utan mehet ki."""
        client.subscribe(self.topic)
        self.get_logger().info(f'MQTT csatlakozva, feliratkozva: {self.topic}')

    def _on_denm(self, client, userdata, msg):
        try:
            sid, seq, lat, lon, speed = parse_denm(
                json.loads(msg.payload.decode('utf-8')))
        except Exception as e:
            self.get_logger().warn(f'DENM feldolgozasi hiba: {e}',
                                   throttle_duration_sec=5.0)
            return

        self.rx += 1

        with self._lock:
            dist = self.front.get(str(sid))
            # Ha az ado nem latszik a trackingben (nem kuld CAM-et), a DENM
            # sajat eventPosition-jebol dontunk.
            if dist is None:
                ref = self._reference()
                if ref is not None:
                    x, y, _ = self.geo.to_map(lat, lon, 0.0)
                    dist = self._ahead(*ref, x, y)

        if dist is None:
            self.get_logger().info(
                f'DENM {sid} ({lat:.7f}, {lon:.7f}) -- nem elottunk',
                throttle_duration_sec=2.0)
            return

        if (sid, seq) in self._seen:
            return
        self._seen.add((sid, seq))

        v = f'{speed:.1f} m/s ({speed * 3.6:.0f} km/h)' if speed is not None \
            else 'nincs sebesseg a DENM-ben'
        self.get_logger().warn(
            f'>>> ELOTTUNK FEKEZ: ID={sid} | {lat:.7f}, {lon:.7f} | '
            f'{dist:.0f} m | fekezesi sebesseg: {v}')

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
        node = DenmReceiver()
    except Exception:
        rclpy.shutdown()
        raise
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
