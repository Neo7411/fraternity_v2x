#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENM vevo - TESZT MOD: csak kiirja a helyzetet, nem fekez.

Menete:
  1. Fogadja a DENM-eket a vanetza/out/denm MQTT topicrol.
  2. Validalja (kotelezo mezok, ertelmes lat/lon, nem lejart, nem sajat magunk).
  3. FOLYAMATOS figyeles: minden esemenyhez (actionId) csuszoablakot tart az
     utolso N pozicioval, es minden ujabb uzenetnel ujraertekel
     (report_period_s-kent riportol, hogy 10 Hz-en ne spammeljen).
  4. Geometria - a relativ vektort a sajat yaw-val elforgatjuk jarmu
     koordinatakba:
       X = hosszirany (+ elore),  Y = oldalirany (+ balra)
     - Elottem van?  X > 0 es a szog a kupon belul.
     - Melyik savban? |Y| <= lane_width/2 -> sajat sav, kulonben masik sav.
     - Szembe jon?   a ket haladasi irany kozotti szog ~180 fok.
     - Szemben ALL?  elottem van + all + szembe nez. Allo jarmu tajolasa a
                     poziciobol elvileg sem szamolhato, ezert:
                       1. a kuldo beteszi a headingjet a DENM location-be
                          (a denm_send.py ezt kuldi) - ez allo autonal is jo;
                       2. ha nincs, a track elmozdulasabol becsuljuk;
                       3. ha most all, az UTOLSO ISMERT iranyat hasznaljuk
                          (amig meg mozgott).
  5. Eredmeny: csak konzol kiiras. Vesz fekezes NINCS ebben a modban.
"""

import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import paho.mqtt.client as mqtt

from nav_msgs.msg import Odometry


LAT_UNAVAIL = 900000001
LON_UNAVAIL = 1800000001
ALT_UNAVAIL = 800001


def tf(src, dst):
    import pyproj
    return pyproj.Transformer.from_crs(src, dst, always_xy=True)


class Wgs84ToEnu:
    """WGS84 -> lokalis ENU, ugyanazzal az origoval mint a cam_receive."""

    def __init__(self, lat0_deg, lon0_deg, h0_m):
        self._lla2ecef = tf(4979, 4978)
        self.x0, self.y0, self.z0 = self._lla2ecef.transform(lon0_deg, lat0_deg, h0_m)
        sl, cl = math.sin(math.radians(lat0_deg)), math.cos(math.radians(lat0_deg))
        so, co = math.sin(math.radians(lon0_deg)), math.cos(math.radians(lon0_deg))
        self.R = np.array([
            [-so,      co,       0.0],
            [-sl * co, -sl * so, cl],
            [ cl * co,  cl * so, sl],
        ])

    def convert(self, lat_deg, lon_deg, h_m):
        x, y, z = self._lla2ecef.transform(lon_deg, lat_deg, h_m)
        enu = self.R @ np.array([x - self.x0, y - self.y0, z - self.z0])
        return float(enu[0]), float(enu[1]), float(enu[2])


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class DenmEebl(Node):
    def __init__(self):
        super().__init__('denm_eebl')

        p = self.declare_parameter
        self.mqtt_broker = p('mqtt_broker', '127.0.0.1').value
        self.mqtt_port = int(p('mqtt_port', 1883).value)
        self.denm_topic = p('mqtt_denm_topic', 'vanetza/out/denm').value

        self.own_station_id = int(p('own_station_id', 1).value)

        # Csuszoablak: ennyi utolso uzenet poziciojat atlagoljuk
        self.sample_size = int(p('sample_size', 3).value)
        # Ennyi ido utan elfelejtjuk az esemenyt (nem jott ujabb uzenet)
        self.event_timeout_s = float(p('event_timeout_s', 5.0).value)
        # Ket kiertekeles kozott legalabb ennyi ido (log spam ellen)
        self.report_period_s = float(p('report_period_s', 0.5).value)

        # Geometria hatarok
        self.max_distance_m = float(p('max_distance_m', 150.0).value)
        self.min_distance_m = float(p('min_distance_m', 1.0).value)
        # "elottunk van" kup fel-szoge fokban
        self.front_cone_deg = float(p('front_cone_deg', 45.0).value)

        # Sav-ellenorzes: a sajat iranyra merolegesen (jarmu Y tengely) ennyi
        # metert tekintunk "egy savnak". |lateral| < lane_width/2 -> sajat sav.
        self.lane_width_m = float(p('lane_width_m', 3.5).value)
        # Ket heading kozotti szog ennyi fok korul -> szembe jon (180 +/- tol)
        self.oncoming_tol_deg = float(p('oncoming_tol_deg', 45.0).value)
        # Ennyi metert kell elmozdulnia a forrasnak, hogy az iranyat becsuljuk
        self.min_track_move_m = float(p('min_track_move_m', 1.0).value)
        # Ez alatt a sebesseg alatt a masik autot allonak tekintjuk
        self.standstill_speed_mps = float(p('standstill_speed_mps', 0.5).value)

        lat0 = float(p('map_origin_lat', 47.5316).value)
        lon0 = float(p('map_origin_lon', 21.6273).value)
        alt0 = float(p('map_origin_alt', 0.0).value)
        self.geo = Wgs84ToEnu(lat0, lon0, alt0)
        self.alt0 = alt0

        # --- sajat pozicio Autoware-bol ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        self.last_odom = None
        self.create_subscription(Odometry, '/localization/kinematic_state', self._on_odom, qos)

        # actionId kulcs -> {'items': csuszoablak, 'stamp':, 'last_report':,
        #                    'track': iranybecsleshez tartott pontok}
        self._events = {}
        self._lock = threading.Lock()

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'MQTT csatlakozasi hiba: {e}')

        self.create_timer(1.0, self._cleanup_tick)
        self.get_logger().info(
            f"TESZT MOD: DENM figyeles '{self.denm_topic}' "
            f"(minta: {self.sample_size} uzenet, kup: {self.front_cone_deg} fok) "
            f"- csak kiiras, nincs fekezes")

    # ------------------------------------------------------------------ ROS

    def _on_odom(self, msg: Odometry):
        self.last_odom = msg

    def _own_state(self):
        """Sajat (e, n, yaw) vagy None."""
        if self.last_odom is None:
            return None
        pp = self.last_odom.pose.pose
        yaw = quat_to_yaw(pp.orientation.x, pp.orientation.y,
                          pp.orientation.z, pp.orientation.w)
        return pp.position.x, pp.position.y, yaw

    # ------------------------------------------------------------------ MQTT

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            client.subscribe(self.denm_topic)
            self.get_logger().info(f'MQTT csatlakozva, feliratkozva: {self.denm_topic}')

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
        except Exception:
            return

        parsed = self._parse_and_validate(data)
        if parsed is None:
            return

        key = parsed['key']
        now = time.time()

        # ENU-ra valtjuk mar itt, hogy a track is ENU-ban legyen
        e, n, _ = self.geo.convert(parsed['lat'], parsed['lon'], parsed['alt'])
        parsed['e'], parsed['n'], parsed['t'] = e, n, now

        with self._lock:
            ev = self._events.setdefault(key, {
                'items': [], 'track': [], 'stamp': now,
                'last_report': 0.0, 'count': 0,
                # utolso ismert irany, amig a forras meg mozgott
                'last_yaw': None, 'last_yaw_t': None, 'last_yaw_src': None,
            })
            ev['stamp'] = now
            ev['count'] += 1

            # csuszoablak: csak az utolso sample_size uzenet
            ev['items'].append(parsed)
            if len(ev['items']) > self.sample_size:
                ev['items'].pop(0)

            # track: az iranybecsleshez hosszabb elozmenyt tartunk
            ev['track'].append((e, n, now))
            if len(ev['track']) > 50:
                ev['track'].pop(0)

            # Az utolso ISMERT iranyt minden uzenetnel frissitjuk (nem csak
            # riportnal), hogy a megallas elotti irany biztosan megmaradjon.
            yaw_now, yaw_src = self._current_src_yaw(parsed, ev['track'])
            if yaw_now is not None:
                ev['last_yaw'] = yaw_now
                ev['last_yaw_t'] = now
                ev['last_yaw_src'] = yaw_src

            # meg nem gyult ossze az elso ablak -> csak jelezzuk
            if len(ev['items']) < self.sample_size:
                self.get_logger().info(
                    f'DENM minta gyul: {key} ({len(ev["items"])}/{self.sample_size})')
                return

            # folyamatos figyeles, de nem minden 10 Hz-es uzenetnel irunk riportot
            if now - ev['last_report'] < self.report_period_s:
                return
            ev['last_report'] = now

            items = list(ev['items'])
            track = list(ev['track'])
            msg_count = ev['count']
            last_yaw = ev['last_yaw']
            last_yaw_t = ev['last_yaw_t']
            last_yaw_src = ev['last_yaw_src']

        self._decide(key, items, track, msg_count,
                     last_yaw, last_yaw_t, last_yaw_src)

    # ------------------------------------------------------- validalas

    def _parse_and_validate(self, data):
        """Kibontja es ellenorzi a DENM-et. None ha nem valid."""
        denm = data.get('fields', {}).get('denm', {})
        if not denm:
            denm = data if 'management' in data else {}
        if not denm:
            return None

        mgmt = denm.get('management', {})
        action = mgmt.get('actionId', {})
        origin_sid = action.get('originatingStationId')
        seq = action.get('sequenceNumber')
        if origin_sid is None or seq is None:
            self.get_logger().warn('Invalid DENM: nincs actionId', throttle_duration_sec=5.0)
            return None

        # sajat uzenet kiszurese
        if int(origin_sid) == self.own_station_id:
            return None

        ev = mgmt.get('eventPosition', {})
        lat = ev.get('latitude')
        lon = ev.get('longitude')
        if lat is None or lon is None:
            self.get_logger().warn('Invalid DENM: nincs eventPosition', throttle_duration_sec=5.0)
            return None
        if lat == LAT_UNAVAIL or lon == LON_UNAVAIL:
            return None

        lat = float(lat)
        lon = float(lon)
        # ETSI 1e-7 fok egesz vagy mar fok
        if abs(lat) > 900:
            lat /= 1e7
        if abs(lon) > 1800:
            lon /= 1e7
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            self.get_logger().warn(
                f'Invalid DENM: ertelmetlen pozicio lat={lat} lon={lon}',
                throttle_duration_sec=5.0)
            return None

        alt_obj = ev.get('altitude', {})
        alt = alt_obj.get('altitudeValue', ALT_UNAVAIL) if isinstance(alt_obj, dict) else alt_obj
        alt = float(alt) if alt is not None else ALT_UNAVAIL
        if alt == ALT_UNAVAIL or abs(alt) > 1e5:
            alt = self.alt0
        elif abs(alt) > 1000:
            alt /= 100.0

        # ervenyesseg: detectionTime + validityDuration
        det = mgmt.get('detectionTime')
        validity = mgmt.get('validityDuration', 0)
        if det is not None:
            det = float(det)
            if det > 1e11:  # ms -> s
                det /= 1000.0
            age = time.time() - det
            if validity and age > float(validity) + self.event_timeout_s:
                self.get_logger().info(
                    f'DENM elavult: age={age:.1f}s validity={validity}s',
                    throttle_duration_sec=5.0)
                return None

        et = denm.get('situation', {}).get('eventType', {})
        cc_scc = et.get('ccAndScc', {}) if isinstance(et, dict) else {}
        is_eebl_cause = any(k.startswith('dangerousSituation') for k in cc_scc) \
            or et.get('causeCode') == 99

        # Opcionalis: ha a kuldo beteszi a sajat headingjet a location containerbe,
        # azt hasznaljuk. A sablonunkban nincs -> ilyenkor None, es a forras
        # iranyat a pozicioja elmozdulasabol becsuljuk (_estimate_src_heading).
        src_heading = None
        loc = denm.get('location', {})
        hdg_obj = loc.get('eventPositionHeading')
        if isinstance(hdg_obj, dict):
            hdg_obj = hdg_obj.get('headingValue')
        if hdg_obj is not None:
            try:
                h = float(hdg_obj)
                if h != 3601:  # HEADING_UNAVAIL
                    if h > 360.0:  # 0.1 fok egesz
                        h /= 10.0
                    src_heading = h % 360.0
            except (TypeError, ValueError):
                src_heading = None

        # Opcionalis sebesseg: ebbol tudjuk, hogy a masik auto ALL-e
        src_speed = None
        spd_obj = loc.get('eventSpeed')
        if isinstance(spd_obj, dict):
            spd_obj = spd_obj.get('speedValue')
        if spd_obj is not None:
            try:
                s = float(spd_obj)
                if s != 16383:  # SPEED_UNAVAIL
                    if s > 200:  # cm/s egesz
                        s /= 100.0
                    src_speed = max(0.0, s)
            except (TypeError, ValueError):
                src_speed = None

        return {
            'key': (int(origin_sid), int(seq)),
            'station_id': int(origin_sid),
            'lat': lat, 'lon': lon, 'alt': alt,
            'is_eebl_cause': is_eebl_cause,
            'cc_scc': cc_scc,
            'src_heading': src_heading,
            'src_speed': src_speed,
        }

    # ------------------------------------------------------- geometria

    def _estimate_src_heading(self, track):
        """A forras haladasi iranya (ENU yaw, rad) a poziciojanak elmozdulasabol.

        A DENM-ben nincs heading, ezert a track legelso es legutolso pontja
        kozotti vektorbol becsuljuk. None, ha alig mozdult (all, vagy zaj).
        """
        if len(track) < 2:
            return None, 0.0
        e0, n0, _ = track[0]
        e1, n1, _ = track[-1]
        de, dn = e1 - e0, n1 - n0
        moved = math.hypot(de, dn)
        if moved < self.min_track_move_m:
            return None, moved
        return math.atan2(dn, de), moved

    def _current_src_yaw(self, parsed, track):
        """A forras MOSTANI iranya (ENU yaw, rad) + honnan tudjuk.

        Elsodlegesen a DENM-ben kuldott heading, mert az allo jarmunel is
        ervenyes. Ha nincs, a pozicio elmozdulasabol becsuljuk - de az csak
        akkor mukodik, ha a masik auto mozog.
        """
        h = parsed.get('src_heading')
        if h is not None:
            # ETSI heading (eszaktol, oramutato szerint) -> ENU yaw
            return math.radians(90.0 - h), 'DENM heading'

        yaw, moved = self._estimate_src_heading(track)
        if yaw is None:
            return None, f'nincs elmozdulas ({moved:.1f} m)'
        return yaw, f'becsult elmozdulasbol ({moved:.1f} m)'

    def _decide(self, key, items, track, msg_count,
                last_yaw=None, last_yaw_t=None, last_yaw_src=None):
        own = self._own_state()
        if own is None:
            self.get_logger().warn('Nincs sajat pozicio, DENM kiertekeles kihagyva',
                                   throttle_duration_sec=2.0)
            return
        own_e, own_n, own_yaw = own

        # csuszoablak atlagolasa ENU-ban (a pontok mar ENU-ban vannak)
        src_e = sum(it['e'] for it in items) / len(items)
        src_n = sum(it['n'] for it in items) / len(items)

        # 1. vektor: ket GNSS pozicio kulonbsege (relativ vektor)
        rel_e = src_e - own_e
        rel_n = src_n - own_n
        dist = math.hypot(rel_e, rel_n)

        # 2. vektor: sajat haladasi irany
        fwd_e = math.cos(own_yaw)
        fwd_n = math.sin(own_yaw)

        if dist < 1e-6:
            angle_deg = 0.0
        else:
            cos_a = (rel_e * fwd_e + rel_n * fwd_n) / dist
            cos_a = max(-1.0, min(1.0, cos_a))
            angle_deg = math.degrees(math.acos(cos_a))

        # A relativ vektor elforgatasa a sajat yaw-val -> jarmu koordinatak.
        # X = hosszirany (+ elore), Y = oldalirany (+ balra). Ez a "sav" teszt.
        veh_x = rel_e * fwd_e + rel_n * fwd_n
        veh_y = -rel_e * fwd_n + rel_n * fwd_e

        in_front = angle_deg <= self.front_cone_deg
        in_range = self.min_distance_m <= dist <= self.max_distance_m

        # --- SAV: az Y (oldalirany) alapjan ---
        half_lane = self.lane_width_m / 2.0
        same_lane = abs(veh_y) <= half_lane
        lane_offset = int(round(veh_y / self.lane_width_m)) if self.lane_width_m > 0 else 0
        if same_lane:
            lane_txt = 'SAJAT SAV (egy sikban)'
        else:
            side = 'bal' if veh_y > 0 else 'jobb'
            lane_txt = f'MASIK SAV ({side}, {abs(lane_offset)} savval)'

        # --- ALL VAGY MOZOG a masik auto? ---
        src_speed = items[-1].get('src_speed')
        if src_speed is not None:
            standing = src_speed < self.standstill_speed_mps
            speed_txt = f'{src_speed:.1f} m/s (DENM eventSpeed)'
        else:
            # Nincs sebesseg a DENM-ben -> a track elmozdulasabol dontunk
            _, moved_now = self._estimate_src_heading(track)
            standing = moved_now < self.min_track_move_m
            speed_txt = f'nincs adat, elmozdulas {moved_now:.1f} m'
        motion_txt = 'ALL' if standing else 'MOZOG'

        # --- MERRE NEZ / MERRE HALAD a masik auto ---
        # Elsodlegesen a mostani irany. Ha all es nincs DENM heading, akkor
        # az utolso ismert iranyat hasznaljuk (amig meg mozgott).
        src_yaw, heading_src = self._current_src_yaw(items[-1], track)
        stale_yaw_age = None
        if src_yaw is None and last_yaw is not None:
            src_yaw = last_yaw
            stale_yaw_age = time.time() - last_yaw_t if last_yaw_t else None
            age_txt = f'{stale_yaw_age:.1f}s' if stale_yaw_age is not None else '?'
            heading_src = f'UTOLSO ISMERT irany {age_txt} regi ({last_yaw_src})'

        if src_yaw is None:
            rel_heading_deg = None
            oncoming = same_dir = facing_me = False
            face_angle = float('nan')
            heading_txt = f'nem meghatarozhato ({heading_src})'
        else:
            d = math.degrees(src_yaw - own_yaw)
            rel_heading_deg = (d + 180.0) % 360.0 - 180.0   # [-180, 180]
            oncoming = abs(abs(rel_heading_deg) - 180.0) <= self.oncoming_tol_deg
            same_dir = abs(rel_heading_deg) <= self.oncoming_tol_deg

            # "Velem szemben van?" - nem eleg, hogy ellentetes iranyba nez:
            # felenk is kell nezzen. A forrasbol felenk mutato vektor es a
            # forras sajat iranya kozotti szog dönt.
            to_me_yaw = math.atan2(-rel_n, -rel_e)   # forrasbol enfelem
            d2 = math.degrees(src_yaw - to_me_yaw)
            face_angle = abs((d2 + 180.0) % 360.0 - 180.0)
            facing_me = face_angle <= self.oncoming_tol_deg

            state = 'szemben ALL' if (standing and oncoming) else \
                    ('SZEMBE JON' if oncoming else
                     ('EGY IRANYBA' if same_dir else 'KERESZTBE'))
            heading_txt = (f'{state} ({rel_heading_deg:+.1f} fok, {heading_src}), '
                           f'felem nez: {facing_me} ({face_angle:.0f} fok)')

        # --- VERDIKT: hol van hozzam kepest ---
        if in_front:
            where = 'ELOTTEM VAN'
        elif angle_deg >= 180.0 - self.front_cone_deg:
            where = 'MOGOTTEM VAN'
        else:
            where = 'OLDALT VAN (' + ('balra' if veh_y > 0 else 'jobbra') + ')'

        # --- SZEMBEN ALL-E VELEM ---
        # Harom feltetel: elottem van, all, es velem ellentetes iranyba nez.
        # Szemben all = elottem + all + ellentetes iranyba nez + felem nez.
        # A "felem nez" azert kell, mert egy oldalra kikanyarodott allo auto
        # nezhet ellentetes iranyba anelkul, hogy velem szemben allna.
        if in_front and standing and oncoming and facing_me:
            lane_note = 'sajat savban' if same_lane else f'de MASIK savban ({veh_y:+.1f} m)'
            facing_txt = f'IGEN - szemben ALL velem, {lane_note}'
        elif in_front and standing:
            if rel_heading_deg is None:
                facing_txt = 'ALL elottem, de az iranya nem ismert'
            elif oncoming and not facing_me:
                facing_txt = ('NEM egeszen - all elottem ellentetes iranyban, '
                              f'de nem felem nez ({face_angle:.0f} fok)')
            else:
                facing_txt = ('NEM - all elottem, de nem szembe nez '
                              f'({rel_heading_deg:+.0f} fok)')
        elif in_front and oncoming:
            facing_txt = 'NEM all - szembe JON (mozog)'
        elif standing:
            facing_txt = f'ALL, de nem elottem ({where})'
        else:
            facing_txt = f'mozog, {where}'

        # --- EEBL relevancia ---
        # Akkor erint minket, ha elottunk van, hataron belul, ES a sajat savunkban.
        # Szembe jovo a masik savban -> nem relevans (elmegy mellettunk).
        if not in_range:
            relevant, why = False, 'tavolsag hataron kivul'
        elif not in_front:
            relevant, why = False, 'nem elottem van'
        elif not same_lane:
            relevant, why = False, f'masik savban ({veh_y:+.1f} m oldalra)'
        elif standing and oncoming:
            relevant, why = True, 'sajat savban SZEMBEN ALL -> utkozes veszely'
        elif standing:
            relevant, why = True, 'sajat savban ALL elottem -> akadaly'
        elif oncoming:
            relevant, why = True, 'sajat savban SZEMBE jon -> frontalis veszely'
        else:
            relevant, why = True, 'sajat savban, elottem'

        print('=' * 68)
        print(f'  DENM esemeny        : stationId={key[0]} seq={key[1]}  '
              f'(#{msg_count}. uzenet)')
        print(f'  Ablak / track       : {len(items)} atlagolva / {len(track)} pont')
        print(f'  Sajat pozicio (ENU) : E={own_e:8.2f}  N={own_n:8.2f}  '
              f'yaw={math.degrees(own_yaw):6.1f} fok')
        print(f'  Esemeny pozicio(ENU): E={src_e:8.2f}  N={src_n:8.2f}')
        print(f'  Relativ vektor      : dE={rel_e:8.2f}  dN={rel_n:8.2f}')
        print(f'  Jarmu koord. (X,Y)  : X={veh_x:+8.2f}  Y={veh_y:+8.2f}   '
              f'(X=elore, Y=balra)')
        print(f'  Tavolsag            : {dist:.1f} m   '
              f'[{self.min_distance_m:.0f}..{self.max_distance_m:.0f} m] -> {in_range}')
        print(f'  Szog a haladashoz   : {angle_deg:.1f} fok  '
              f'(kup: {self.front_cone_deg:.0f} fok)')
        print(f'  Sav (Y alapjan)     : {lane_txt}   '
              f'[sav={self.lane_width_m:.1f} m, fel={half_lane:.2f} m]')
        print(f'  Masik auto mozgasa  : {motion_txt}  [{speed_txt}]')
        print(f'  Masik auto iranya   : {heading_txt}')
        print(f'  Cause code EEBL     : {items[-1]["is_eebl_cause"]}')
        print(f'  >>> HOL VAN         : {where}')
        print(f'  >>> SZEMBEN ALL?    : {facing_txt}')
        print(f'  >>> EEBL RELEVANS   : {relevant}  ({why})')
        print('=' * 68)

        if standing:
            onc = 'szemben all' if oncoming else 'all'
        elif oncoming:
            onc = 'szembe jon'
        elif same_dir:
            onc = 'egyirany'
        else:
            onc = 'kereszt'
        self.get_logger().info(
            f'DENM {key}: {where} | {dist:.1f}m X={veh_x:+.1f} Y={veh_y:+.1f} | '
            f'{"sajat sav" if same_lane else "masik sav"} | {onc} | '
            f'relevans={relevant}')

    # ------------------------------------------------------- housekeeping

    def _cleanup_tick(self):
        """Lejart esemenyek eldobasa: nem jott ujabb uzenet event_timeout_s-ig."""
        now = time.time()
        with self._lock:
            stale = [k for k, s in self._events.items()
                     if now - s['stamp'] > self.event_timeout_s]
            for k in stale:
                cnt = self._events[k]['count']
                self.get_logger().info(
                    f'DENM esemeny vege: {k} (osszesen {cnt} uzenet), track torolve')
                del self._events[k]

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
    node = DenmEebl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
