#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENM vevo -> EEBL veszfekezes, lanelet2 HD terkep alapu savdontessel.

Matek (ugyanaz mint a lane_check_lanelet2.py-ban):
  EGO lanelet -> following() menten osszefuzott lanc -> LaneletSequence.centerline
  toArcCoordinates() -> (s, d) az EGO-ra es a DENM esemenyre
  delta_s = obj.s - ego.s   ->  elottunk / mogottunk (iv menten)
  d_rel   = obj.d - ego.d   ->  savoffset (0 = sajat sav)

Fekezes ha: elottunk + hataron belul + sajat sav + sajat utvonal.

PROJEKCIO -- a terkep map_projector_info.yaml-jaben allo tipust kell megadni:
  local      Autoware 'Local': a node-oknak NINCS lat/lon-juk, a koordinata a
             local_x / local_y tag-ben all. A lanelet2 sajat projektorai ezt
             nem ismerik (mindent (0,0)-ba tennenek), ezert betoltes utan
             kezzel visszairjuk a pontokat -- lasd load_local_tag_map().
  local_cartesian / mgrs / utm   valodi lat/lon-os terkep, ilyenkor a
             map_origin_lat/lon-nak egyeznie kell a terkep origojaval.
"""

import json
import math
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import paho.mqtt.client as mqtt
import rclpy
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

import lanelet2
from lanelet2.core import BasicPoint2d, LaneletSequence
from lanelet2.geometry import toArcCoordinates, to2D, findNearest
from lanelet2.io import Origin

from nav_msgs.msg import Odometry
from tier4_external_api_msgs.srv import SetEmergency

LAT_UNAVAIL, LON_UNAVAIL, ALT_UNAVAIL = 900000001, 1800000001, 800001
CHAIN_FWD, CHAIN_BACK = 200, 5          # lanc hossza lanelet-ben
HEADING_TOL = math.radians(90.0)        # ennyin belul fogadjuk el az EGO savot


class Wgs84ToEnu:
    """WGS84 -> lokalis ENU. Ugyanaz a matek mint a LocalCartesianProjector."""

    def __init__(self, lat0, lon0, h0):
        self._t = Transformer.from_crs(4979, 4978, always_xy=True)
        self.x0, self.y0, self.z0 = self._t.transform(lon0, lat0, h0)
        sl, cl = math.sin(math.radians(lat0)), math.cos(math.radians(lat0))
        so, co = math.sin(math.radians(lon0)), math.cos(math.radians(lon0))
        self.R = np.array([[-so, co, 0.0],
                           [-sl * co, -sl * so, cl],
                           [cl * co, cl * so, sl]])

    def convert(self, lat, lon, h):
        x, y, z = self._t.transform(lon, lat, h)
        e, n, u = self.R @ np.array([x - self.x0, y - self.y0, z - self.z0])
        return float(e), float(n), float(u)


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def lane_offset_idx(d_rel, width):
    """Determinisztikus kerekites (round() savhataron megjosolhatatlan)."""
    return int(math.floor(d_rel / width + 0.5)) if width > 0 else 0


def load_local_tag_map(osm_path):
    """Autoware 'Local' projekcioju terkep betoltese.

    Ilyen terkepben a node-ok lat/lon attributuma URES, a valodi koordinata a
    local_x / local_y (es ele) tag-ben all. A lanelet2 egyik projektora sem
    ismeri ezt a konvenciot: az ures lat/lon-t 0-nak veszi, igy MINDEN pont
    (0,0)-ba kerul -- ekkor a findNearest sosem talalja meg az EGO savjat.

    Ezert a terkepet ugy toltjuk be, ahogy tudjuk, majd a pontok koordinatait
    visszairjuk az OSM tag-ekbol. A pontok modositasa a laneletLayer
    centerline-jaira es a findNearest terbeli indexere is atvezetodik.
    """
    lmap = lanelet2.io.load(osm_path, lanelet2.projection.LocalCartesianProjector(
        Origin(0.0, 0.0, 0.0)))

    local = {}
    for node in ET.parse(osm_path).getroot().findall('node'):
        tags = {t.get('k'): t.get('v') for t in node.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            local[int(node.get('id'))] = (float(tags['local_x']),
                                          float(tags['local_y']),
                                          float(tags.get('ele', 0.0) or 0.0))
    if not local:
        raise RuntimeError(
            f"{osm_path}: nincs local_x/local_y tag -- ez nem 'Local' "
            f"projekcioju terkep, adj meg mas lanelet2_projector erteket")

    missing = 0
    for pt in lmap.pointLayer:
        xyz = local.get(pt.id)
        if xyz is None:
            missing += 1
            continue
        pt.x, pt.y, pt.z = xyz
    if missing:
        raise RuntimeError(f'{osm_path}: {missing} pontnak nincs local_x/local_y tagje')
    return lmap


def seg_yaw_near(pts, x, y):
    """A polyline iranya a (x,y)-hoz legkozelebbi szegmensen."""
    best_d2, best_yaw = float('inf'), None
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            continue
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / l2))
        d2 = (x - x1 - t * dx) ** 2 + (y - y1 - t * dy) ** 2
        if d2 < best_d2:
            best_d2, best_yaw = d2, math.atan2(dy, dx)
    return best_yaw


class LaneChecker:
    """HD terkep alapu Frenet savdontes. A lanc cache-elt: csak akkor epul
    ujra, ha az EGO atlep masik lanelet-be (10 Hz-en nem birna)."""

    def __init__(self, log, osm, projector, lat0, lon0, alt0,
                 lane_width, search_radius):
        self.log = log
        self.lane_width = lane_width
        self.search_radius = search_radius
        if projector == 'local':
            self.map = load_local_tag_map(osm)
        else:
            self.map = lanelet2.io.load(
                osm, self._projector(projector, lat0, lon0, alt0))
        rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = lanelet2.routing.RoutingGraph(self.map, rules)
        self._pts_cache = {}
        self._chain = None
        self._miss = ''
        xs = [p.x for p in self.map.pointLayer]
        ys = [p.y for p in self.map.pointLayer]
        self.extent = (min(xs), max(xs), min(ys), max(ys)) if xs else None
        log.info(f'terkep: {osm} ({len(self.map.laneletLayer)} lanelet, '
                 f'{projector}, origo {lat0},{lon0})')
        if self.extent:
            log.info('terkep kiterjedes: x %.1f..%.1f  y %.1f..%.1f m'
                     % self.extent)

    @staticmethod
    def _projector(kind, lat, lon, alt):
        o, P = Origin(lat, lon, alt), lanelet2.projection
        if kind == 'mgrs':
            return P.MGRSProjector(o)
        if kind == 'utm':
            return P.UtmProjector(o)
        if kind == 'local_cartesian':
            return P.LocalCartesianProjector(o)
        raise RuntimeError(f"ismeretlen lanelet2_projector: '{kind}' "
                           f"(local | local_cartesian | mgrs | utm)")

    def _pts(self, llt):
        if llt.id not in self._pts_cache:
            self._pts_cache[llt.id] = [(p.x, p.y) for p in llt.centerline]
        return self._pts_cache[llt.id]

    def _find(self, x, y, yaw=None):
        """Melyik lanelet-ben van a pont. yaw eseten csak az egyezo iranyuak
        jonnek szoba -- kulonben az ellenkezo iranyu savot valasztana.
        A findNearest tavolsag szerint rendezve ad vissza, igy az elso
        elfogadott talalat egyben a legkozelebbi is."""
        near = findNearest(self.map.laneletLayer, BasicPoint2d(x, y), 8)
        for dist, llt in near:
            if dist > self.search_radius:
                break
            if yaw is not None:
                lyaw = seg_yaw_near(self._pts(llt), x, y)
                if lyaw is None or abs((lyaw - yaw + math.pi)
                                       % (2 * math.pi) - math.pi) > HEADING_TOL:
                    continue
            return llt

        # nem volt talalat -- a naplo mondja meg, melyik ok miatt
        if near:
            d0, l0 = near[0]
            self._miss = (f'legkozelebbi lanelet {l0.id} {d0:.1f} m-re '
                          f'(sugar {self.search_radius} m)' if d0 > self.search_radius
                          else f'lanelet {l0.id} {d0:.1f} m-re van, de az iranya nem egyezik')
        else:
            self._miss = 'ures terkep'
        return None

    def _chain_for(self, llt):
        if self._chain and self._chain['id'] == llt.id:
            return self._chain

        fwd, seen = [], set()
        cur = llt
        while cur is not None and cur.id not in seen and len(seen) < CHAIN_FWD:
            seen.add(cur.id)
            fwd.append(cur)
            nxt = self.graph.following(cur)
            cur = nxt[0] if nxt else None

        back, cur = [], llt          # hatra is, hogy a mogottunk levo esemenyre
        for _ in range(CHAIN_BACK):  # ne 0-ra vagott s jojjon ki
            prev = self.graph.previous(cur)
            if not prev or prev[0].id in seen:
                break
            cur = prev[0]
            seen.add(cur.id)
            back.append(cur)
        back.reverse()

        llts = back + fwd
        centerline = to2D(LaneletSequence(llts).centerline)

        route = {l.id for l in llts}   # + oldalso szomszedok (savvaltas)
        for l in llts:
            for fn in ('lefts', 'rights', 'adjacentLefts', 'adjacentRights'):
                try:
                    route.update(n.id for n in getattr(self.graph, fn)(l))
                except Exception:
                    pass

        self._chain = {'id': llt.id, 'centerline': centerline,
                       'route': route, 'n': len(llts)}
        self.log.info(f'EGO lanelet {llt.id} -> lanc {len(llts)} lanelet')
        return self._chain

    def check(self, ego_x, ego_y, ego_yaw, obj_x, obj_y):
        """dict vagy None, ha nem ertelmezheto."""
        ego_llt = self._find(ego_x, ego_y, ego_yaw)
        if ego_llt is None:
            self.log.warn(f'EGO ({ego_x:.1f}, {ego_y:.1f}) nincs egyetlen savon '
                          f'sem: {self._miss}', throttle_duration_sec=5.0)
            return None

        ch = self._chain_for(ego_llt)
        ego = toArcCoordinates(ch['centerline'], BasicPoint2d(ego_x, ego_y))
        obj = toArcCoordinates(ch['centerline'], BasicPoint2d(obj_x, obj_y))
        d_rel = obj.distance - ego.distance
        off = lane_offset_idx(d_rel, self.lane_width)
        obj_llt = self._find(obj_x, obj_y)

        return {
            'ego_id': ego_llt.id, 'obj_id': obj_llt.id if obj_llt else None,
            'ego_s': ego.length, 'ego_d': ego.distance,
            'obj_s': obj.length, 'obj_d': obj.distance,
            'delta_s': obj.length - ego.length, 'd_rel': d_rel,
            'offset': off, 'same_lane': off == 0, 'n': ch['n'],
            'on_route': obj_llt is not None and obj_llt.id in ch['route'],
        }


class DenmEebl(Node):
    def __init__(self):
        super().__init__('denm_eebl')
        p = self.declare_parameter

        self.denm_topic = p('mqtt_denm_topic', 'vanetza/out/denm').value
        self.own_station_id = int(p('own_station_id', 1).value)
        self.sample_size = int(p('sample_size', 3).value)
        self.event_timeout_s = float(p('event_timeout_s', 5.0).value)
        self.report_period_s = float(p('report_period_s', 0.5).value)
        self.min_distance_m = float(p('min_distance_m', 1.0).value)
        self.max_distance_m = float(p('max_distance_m', 150.0).value)
        self.lane_width_m = float(p('lane_width_m', 3.5).value)
        self.require_eebl_cause = bool(p('require_eebl_cause', False).value)
        self.emergency_srv = p('emergency_service', '/api/autoware/set/emergency').value

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        self.alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = Wgs84ToEnu(lat0, lon0, self.alt0)

        osm = str(p('lanelet2_map_path', '/home/aw/maps/highway/lanelet2_map.osm').value)
        if not osm:
            raise RuntimeError('lanelet2_map_path parameter kotelezo')
        self.lanes = LaneChecker(
            self.get_logger(), osm, str(p('lanelet2_projector', 'local').value),
            lat0, lon0, self.alt0, self.lane_width_m,
            float(p('lane_search_radius_m', 5.0).value))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.last_odom = None
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 self._on_odom, qos)

        self._events = {}
        self._braked = set()
        self._lock = threading.Lock()

        self.cli = self.create_client(SetEmergency, self.emergency_srv)
        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = lambda c, u, f, rc, pr: (
            c.subscribe(self.denm_topic) if rc == 0 else None)
        self.mqtt.on_message = self._on_message
        try:
            self.mqtt.connect(str(p('mqtt_broker', '127.0.0.1').value),
                              int(p('mqtt_port', 1883).value), 60)
            self.mqtt.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT hiba: {e}')

        self.create_timer(1.0, self._cleanup)
        self.get_logger().info(
            f"DENM figyeles '{self.denm_topic}' | "
            f"{self.emergency_srv} | "
            f"sav={self.lane_width_m} m, tav<={self.max_distance_m} m")

    # ---------------------------------------------------------------- input

    def _on_odom(self, msg):
        self.last_odom = msg

    def _on_message(self, client, userdata, msg):
        try:
            parsed = self._parse(json.loads(msg.payload.decode('utf-8')))
        except Exception as e:
            self.get_logger().warn(f'Rossz payload: {e}', throttle_duration_sec=5.0)
            return
        if parsed is None:
            return

        now = time.time()
        e, n, _ = self.geo.convert(parsed['lat'], parsed['lon'], parsed['alt'])
        parsed['e'], parsed['n'] = e, n
        key = parsed['key']

        with self._lock:
            ev = self._events.setdefault(key, {'items': [], 'stamp': now,
                                               'report': 0.0, 'count': 0})
            ev['stamp'], ev['count'] = now, ev['count'] + 1
            ev['items'].append(parsed)
            if len(ev['items']) > self.sample_size:
                ev['items'].pop(0)

            if len(ev['items']) < self.sample_size:
                return
            if now - ev['report'] < self.report_period_s:
                return
            ev['report'] = now
            items, count = list(ev['items']), ev['count']

        self._decide(key, items, count)

    def _parse(self, data):
        """Kibontas + validalas. None ha eldobjuk."""
        denm = data.get('fields', {}).get('denm') or (
            data if 'management' in data else None)
        if not denm:
            return None

        mgmt = denm.get('management', {})
        act = mgmt.get('actionId', {})
        sid, seq = act.get('originatingStationId'), act.get('sequenceNumber')
        if sid is None or seq is None or int(sid) == self.own_station_id:
            return None

        pos = mgmt.get('eventPosition', {})
        lat, lon = pos.get('latitude'), pos.get('longitude')
        if lat is None or lon is None or lat == LAT_UNAVAIL or lon == LON_UNAVAIL:
            return None
        lat, lon = float(lat), float(lon)
        if abs(lat) > 900:      # ETSI 1e-7 fok egesz
            lat /= 1e7
        if abs(lon) > 1800:
            lon /= 1e7
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            self.get_logger().warn(f'Ertelmetlen pozicio {lat},{lon}',
                                   throttle_duration_sec=5.0)
            return None

        alt = pos.get('altitude', {})
        alt = alt.get('altitudeValue') if isinstance(alt, dict) else alt
        alt = float(alt) if alt is not None else ALT_UNAVAIL
        if alt == ALT_UNAVAIL or abs(alt) > 1e5:
            alt = self.alt0
        elif abs(alt) > 1000:   # cm -> m
            alt /= 100.0

        det, validity = mgmt.get('detectionTime'), mgmt.get('validityDuration', 0)
        if det is not None and validity:
            det = float(det)
            if det > 1e11:      # ms -> s
                det /= 1000.0
            if time.time() - det > float(validity) + self.event_timeout_s:
                return None

        et = denm.get('situation', {}).get('eventType', {})
        cc = et.get('ccAndScc', {}) if isinstance(et, dict) else {}
        eebl = any(k.startswith('dangerousSituation') for k in cc) \
            or et.get('causeCode') == 99

        return {'key': (int(sid), int(seq)), 'lat': lat, 'lon': lon,
                'alt': alt, 'eebl': eebl}

    # --------------------------------------------------------------- dontes

    def _decide(self, key, items, count):
        if self.last_odom is None:
            self.get_logger().warn('Nincs sajat pozicio', throttle_duration_sec=2.0)
            return
        pp = self.last_odom.pose.pose
        ego_x, ego_y = pp.position.x, pp.position.y
        ego_yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y,
                              pp.orientation.z, pp.orientation.w)

        obj_x = sum(i['e'] for i in items) / len(items)   # GNSS zaj atlagolas
        obj_y = sum(i['n'] for i in items) / len(items)

        r = self.lanes.check(ego_x, ego_y, ego_yaw, obj_x, obj_y)
        if r is None:
            return

        gap = r['delta_s']
        # A savdontes megy elore: az "on_route" olyan terkepen, aminek nincs
        # lefts/rights relacioja, gyakorlatilag "ugyanaz a lanelet"-et jelent,
        # es a szomszed savra felrevezeto indoklast adna.
        if not r['same_lane']:
            side = 'balra' if r['offset'] > 0 else 'jobbra'
            relevant, why = False, f'{abs(r["offset"])} savval {side} ({r["d_rel"]:+.1f} m)'
        elif not r['on_route']:
            relevant, why = False, f'nem a sajat utvonalunkon (lanelet {r["obj_id"]})'
        elif gap < 0:
            relevant, why = False, f'mogottunk ({gap:.1f} m)'
        elif not (self.min_distance_m <= gap <= self.max_distance_m):
            relevant, why = False, f'tavolsag hataron kivul ({gap:.1f} m)'
        elif self.require_eebl_cause and not items[-1]['eebl']:
            relevant, why = False, 'nem EEBL cause code'
        else:
            relevant, why = True, f'sajat savban, {gap:.1f} m elottunk'

        self.get_logger().info(
            f'DENM {key} #{count} | lanelet {r["ego_id"]}->{r["obj_id"]} '
            f'(lanc {r["n"]}) | s: {r["ego_s"]:.1f}->{r["obj_s"]:.1f} '
            f'delta_s={gap:+.1f} m | d: {r["ego_d"]:+.2f}->{r["obj_d"]:+.2f} '
            f'd_rel={r["d_rel"]:+.2f} m off={r["offset"]:+d} | '
            f'relevans={relevant} ({why})')

        if not relevant:
            return
        with self._lock:
            if key in self._braked:      # egy esemenyre csak egyszer
                return
            self._braked.add(key)
        self._brake()

    # -------------------------------------------------------------- fekezes

    def _brake(self):
        if not self.cli.service_is_ready():
            self.get_logger().error(f'{self.emergency_srv} nem elerheto!')
            return
        req = SetEmergency.Request()
        req.emergency = True
        self.cli.call_async(req).add_done_callback(self._on_brake_response)
        self.get_logger().warn('EEBL -> VESZFEKEZES kerese elkuldve')

    def _on_brake_response(self, future):
        try:
            st = future.result().status          # SUCCESS=1 IGNORED=2 WARN=3 ERROR=4
        except Exception as e:
            self.get_logger().error(f'Veszfek hivas hiba: {e}')
            return
        if st.code in (1, 3):
            self.get_logger().warn(f'VESZFEKEZES AKTIV ({st.message})')
        else:
            self.get_logger().error(f'Autoware nem fekezett: {st.code} {st.message}')

    def _cleanup(self):
        now = time.time()
        with self._lock:
            for k in [k for k, v in self._events.items()
                      if now - v['stamp'] > self.event_timeout_s]:
                self.get_logger().info(f'DENM esemeny vege: {k} '
                                       f'({self._events[k]["count"]} uzenet)')
                del self._events[k]
                self._braked.discard(k)   # ha ujra felbukkan, az UJ esemeny

    def destroy_node(self):
        try:
            self.mqtt.loop_stop()
            self.mqtt.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DenmEebl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass          # Ctrl-C vagy SIGTERM -- nem hiba
    finally:
        node.destroy_node()
        if rclpy.ok():            # SIGTERM eseten a context mar le van allitva
            rclpy.shutdown()


if __name__ == '__main__':
    main()