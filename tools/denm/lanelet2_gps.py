#!/usr/bin/env python3
"""
lane_check_lanelet2.py -- savdontes teszt a HIVATALOS lanelet2 toArcCoordinates
fuggvennyel. Ugyanazt a matekot csinalja, mint a lane_check.py (sajat Frenet-
vetites), de az igazi konyvtarral -- ez a validacio: ha a ketto egyezik, mind
a ket implementacio helyes.

FONTOS -- csak teszthez: a projektor origoja (0,0)-ra van allitva, es a
terkepet is (0,0) origoval kell generalni (make_highway_map.py --lat 0 --lon 0).
Igy a projekcio kerdese teljesen kiesik: nem szamit, "hol van" a terkep,
csak az, hogy a projektor es a generator origoja EGYEZZEN.

Hasznalat:
  python3 lane_check_lanelet2.py --osm highway0/lanelet2_map.osm --list
  python3 lane_check_lanelet2.py --osm highway0/lanelet2_map.osm \
      --lane 248 --ego-x 300 --ego-y -2.25 --pt-x 450 --pt-y -5.75
"""
import argparse
import math
import xml.etree.ElementTree as ET

import lanelet2
from lanelet2.core import BasicPoint2d, LaneletSequence
from lanelet2.geometry import toArcCoordinates, to2D
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector


def load_local_tag_map(osm_path):
    """Autoware 'Local' projekcioju terkep: a valodi koordinata a local_x /
    local_y tag-ben all, a node lat/lon attributuma ures. Betoltes utan
    visszairjuk a pontokat -- ugyanaz, mint a denm_rec_lanelet2.py-ban."""
    lmap = lanelet2.io.load(osm_path, LocalCartesianProjector(Origin(0.0, 0.0)))

    local = {}
    for node in ET.parse(osm_path).getroot().findall("node"):
        tags = {t.get("k"): t.get("v") for t in node.findall("tag")}
        if "local_x" in tags and "local_y" in tags:
            local[int(node.get("id"))] = (float(tags["local_x"]),
                                          float(tags["local_y"]),
                                          float(tags.get("ele", 0.0) or 0.0))
    if not local:
        raise SystemExit(f"{osm_path}: nincs local_x/local_y tag -- "
                         f"probald a --projector local_cartesian kapcsolot")

    for pt in lmap.pointLayer:
        if pt.id in local:
            pt.x, pt.y, pt.z = local[pt.id]
    return lmap


def lane_offset(d_rel, width=3.5):
    """Determinisztikus kerekites -- a sima round() savhataron megjosolhatatlan
    (Python banker's rounding: round(-0.5) == 0)."""
    return int(math.floor(d_rel / width + 0.5))


def build_chain_centerline(lanelet_map, routing_graph, start_id, max_len=200):
    """A savat vegigfuzi a routing graph following() relacioja menten, majd
    a hivatalos LaneletSequence.centerline-t hasznalja -- ez mar maga kezeli
    a szomszedos lanelet-ek kozotti atmenetet, nem kell kezi dedup."""
    llt = lanelet_map.laneletLayer[start_id]
    seq, visited = [], set()
    while llt is not None and llt.id not in visited and len(visited) < max_len:
        visited.add(llt.id)
        seq.append(llt)
        following = routing_graph.following(llt)
        llt = following[0] if following else None

    sequence = LaneletSequence(seq)
    centerline_2d = to2D(sequence.centerline)
    return centerline_2d, len(seq)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--osm", required=True)
    ap.add_argument("--projector", default="local",
                    choices=["local", "local_cartesian"],
                    help="local = Autoware 'Local' (local_x/local_y tag-ek)")
    ap.add_argument("--lane", type=int)
    ap.add_argument("--width", type=float, default=3.5)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--ego-x", type=float)
    ap.add_argument("--ego-y", type=float)
    ap.add_argument("--pt-x", type=float)
    ap.add_argument("--pt-y", type=float)
    a = ap.parse_args()

    # A projektor a terkep map_projector_info.yaml-jat kell kovesse:
    #   local            Autoware 'Local' -- a node-oknak nincs lat/lon-juk,
    #                    a koordinata a local_x / local_y tag-ben all. Ezt
    #                    egyik lanelet2 projektor sem ismeri (mindent (0,0)-ba
    #                    tenne), ezert betoltes utan visszairjuk a pontokat.
    #   local_cartesian  valodi lat/lon-os terkep, (0,0) origoval generalva.
    if a.projector == "local":
        lanelet_map = load_local_tag_map(a.osm)
    else:
        lanelet_map = lanelet2.io.load(a.osm,
                                       LocalCartesianProjector(Origin(0.0, 0.0)))

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle)
    routing_graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)

    print(f"terkep: {len(lanelet_map.laneletLayer)} lanelet betoltve")

    if a.list or not a.lane:
        print("\nsavkezdetek (azok a lanelet-ek, amiknek nincs 'previous'-a):")
        for llt in lanelet_map.laneletLayer:
            if not routing_graph.previous(llt):
                _, n = build_chain_centerline(lanelet_map, routing_graph, llt.id)
                print(f"  lanelet {llt.id:>5}  lanc hossza: {n} szegmens")
        if not a.lane:
            return

    centerline, n_seg = build_chain_centerline(lanelet_map, routing_graph, a.lane)
    print(f"\nEGO sav: lanelet {a.lane} -> {n_seg} lanelet osszefuzve "
          f"(LaneletSequence.centerline)")

    if None in (a.ego_x, a.ego_y, a.pt_x, a.pt_y):
        print("adj meg --ego-x/--ego-y/--pt-x/--pt-y erteket a dontes teszthez")
        return

    ego_arc = toArcCoordinates(centerline, BasicPoint2d(a.ego_x, a.ego_y))
    obj_arc = toArcCoordinates(centerline, BasicPoint2d(a.pt_x, a.pt_y))

    delta_s = obj_arc.length - ego_arc.length
    d_rel = obj_arc.distance - ego_arc.distance
    off = lane_offset(d_rel, a.width)
    decision = ("UGYANAZ A SAV" if off == 0
                else f"{abs(off)} savval {'BALRA' if off > 0 else 'JOBBRA'}")

    print(f"\nEGO Frenet: s={ego_arc.length:8.2f} m  d={ego_arc.distance:+7.3f} m")
    print(f"OBJ Frenet: s={obj_arc.length:8.2f} m  d={obj_arc.distance:+7.3f} m")
    print(f"delta_s = {delta_s:+8.2f} m  ({'elottunk' if delta_s >= 0 else 'mogottunk'})")
    print(f"d_rel   = {d_rel:+8.2f} m")
    print(f"lane_offset = {off:+d}   maradek {d_rel - off * a.width:+.2f} m")
    print(f"dontes  = {decision}")


if __name__ == "__main__":
    main()
