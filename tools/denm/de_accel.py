#!/usr/bin/env python3
"""
brake_monitor.py — Autoware fékezés-analizátor

Feliratkozik a lassulasra, a sebessegre es a control parancsra, detektalja
a fekezesi epizodokat, es epizodonkent kiirja:
  - maximalis lassulas (m/s^2 es g)
  - atlagos lassulas
  - parancsolt vs. tenyleges lassulas kulonbsege
  - reakcioido (parancs -> tenyleges lassulas felfutasa)
  - fekut es sebessegcsokkenes

Hasznalat:
    python3 brake_monitor.py                  # csak epizod-osszegzes
    python3 brake_monitor.py --live           # elo kiiras is minden mintanal
    python3 brake_monitor.py --threshold -1.0 # sajat kuszob (m/s^2)
    python3 brake_monitor.py --csv out.csv    # mintak mentese CSV-be
"""

import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import AccelWithCovarianceStamped

try:
    from autoware_vehicle_msgs.msg import VelocityReport
except ImportError:  # regebbi Autoware
    from autoware_auto_vehicle_msgs.msg import VelocityReport

CONTROL_MSG = None
try:
    from autoware_control_msgs.msg import Control as CONTROL_MSG
except ImportError:
    try:
        from autoware_auto_control_msgs.msg import AckermannControlCommand as CONTROL_MSG
    except ImportError:
        pass

G = 9.80665


def qos():
    """BEST_EFFORT subscriber RELIABLE publishertol is kap adatot, forditva nem."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Sample:
    __slots__ = ("t", "accel", "vel", "cmd_accel")

    def __init__(self, t, accel, vel, cmd_accel):
        self.t = t
        self.accel = accel
        self.vel = vel
        self.cmd_accel = cmd_accel


class BrakeMonitor(Node):
    def __init__(self, args):
        super().__init__("brake_monitor")

        self.start_th = args.threshold        # epizod indul, ha accel ez ala megy
        self.end_th = args.threshold / 2.0    # hiszterezis a vegehez
        self.min_duration = args.min_duration
        self.live = args.live

        self.velocity = 0.0
        self.cmd_accel = float("nan")
        self.cmd_valid = False
        self.last_cmd_brake_time = None       # mikor kert eloszor fekezest a control

        self.in_episode = False
        self.samples = []
        self.episode_count = 0

        self.csv_writer = None
        self.csv_file = None
        if args.csv:
            self.csv_file = open(args.csv, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["episode", "t_rel", "accel_mps2", "cmd_accel_mps2", "velocity_mps"]
            )

        self.create_subscription(
            AccelWithCovarianceStamped, args.accel_topic, self.on_accel, qos()
        )
        self.create_subscription(
            VelocityReport, args.velocity_topic, self.on_velocity, qos()
        )
        if CONTROL_MSG is not None:
            self.create_subscription(
                CONTROL_MSG, args.control_topic, self.on_control, qos()
            )
        else:
            self.get_logger().warn(
                "Control uzenettipus nem importalhato — parancsolt lassulas nem lesz."
            )

        self.get_logger().info(
            f"Figyelem: {args.accel_topic} | kuszob: {self.start_th:.2f} m/s^2"
        )

    # ---------- callbackek ----------

    def on_velocity(self, msg):
        self.velocity = float(msg.longitudinal_velocity)

    def on_control(self, msg):
        lon = msg.longitudinal
        # Az acceleration mezo opcionalis: az is_defined_acceleration flag dont.
        defined = getattr(lon, "is_defined_acceleration", True)
        self.cmd_valid = bool(defined)
        self.cmd_accel = float(lon.acceleration) if defined else float("nan")

        if self.cmd_valid and self.cmd_accel < self.start_th:
            if self.last_cmd_brake_time is None:
                self.last_cmd_brake_time = self.now()
        else:
            if not self.in_episode:
                self.last_cmd_brake_time = None

    def on_accel(self, msg):
        t = self.stamp_to_sec(msg.header.stamp) or self.now()
        a = float(msg.accel.accel.linear.x)

        s = Sample(t, a, self.velocity, self.cmd_accel)

        if not self.in_episode:
            if a < self.start_th:
                self.in_episode = True
                self.samples = [s]
                self.episode_count += 1
                self.get_logger().info(f"--- Fekezes #{self.episode_count} indul ---")
            return

        self.samples.append(s)

        if self.live:
            print(
                f"  t={t - self.samples[0].t:6.2f}s  "
                f"a={a:7.3f} m/s^2 ({a / G:6.3f} g)  "
                f"cmd={s.cmd_accel:7.3f}  v={s.vel:6.2f} m/s",
                flush=True,
            )

        if a > self.end_th:
            self.finish_episode()

    # ---------- kiertekeles ----------

    def finish_episode(self):
        self.in_episode = False
        samples = self.samples
        self.samples = []

        if len(samples) < 2:
            self.last_cmd_brake_time = None
            return

        t0 = samples[0].t
        duration = samples[-1].t - t0
        if duration < self.min_duration:
            self.last_cmd_brake_time = None
            return

        accels = [s.accel for s in samples]
        a_max = min(accels)                       # legnegativabb = legerosebb fek
        a_mean = sum(accels) / len(accels)
        i_max = accels.index(a_max)
        t_to_max = samples[i_max].t - t0

        v_start = samples[0].vel
        v_end = samples[-1].vel
        distance = self.integrate_distance(samples)

        cmds = [s.cmd_accel for s in samples if not math.isnan(s.cmd_accel)]
        cmd_max = min(cmds) if cmds else None

        reaction = None
        if self.last_cmd_brake_time is not None:
            reaction = t0 - self.last_cmd_brake_time

        print("")
        print(f"=== Fekezesi epizod #{self.episode_count} ===")
        print(f"  Idotartam            : {duration:.2f} s")
        print(f"  Max lassulas         : {a_max:.3f} m/s^2  ({abs(a_max) / G:.3f} g)")
        print(f"  Atlagos lassulas     : {a_mean:.3f} m/s^2  ({abs(a_mean) / G:.3f} g)")
        print(f"  Felfutas maxig       : {t_to_max:.2f} s")
        if cmd_max is not None:
            print(f"  Max parancsolt       : {cmd_max:.3f} m/s^2")
            print(f"  Kovetesi hiba        : {a_max - cmd_max:+.3f} m/s^2")
        if reaction is not None and reaction > 0:
            print(f"  Parancs -> valos kesl: {reaction:.3f} s")
        print(f"  Sebesseg             : {v_start:.2f} -> {v_end:.2f} m/s "
              f"({v_start * 3.6:.1f} -> {v_end * 3.6:.1f} km/h)")
        print(f"  Megtett ut           : {distance:.2f} m")
        print("")

        if self.csv_writer:
            for s in samples:
                self.csv_writer.writerow(
                    [self.episode_count, f"{s.t - t0:.4f}", f"{s.accel:.4f}",
                     "" if math.isnan(s.cmd_accel) else f"{s.cmd_accel:.4f}",
                     f"{s.vel:.4f}"]
                )
            self.csv_file.flush()

        self.last_cmd_brake_time = None

    @staticmethod
    def integrate_distance(samples):
        """Trapez-integral a sebessegre."""
        d = 0.0
        for prev, cur in zip(samples, samples[1:]):
            dt = cur.t - prev.t
            if 0.0 < dt < 1.0:
                d += (prev.vel + cur.vel) * 0.5 * dt
        return d

    @staticmethod
    def stamp_to_sec(stamp):
        sec = stamp.sec + stamp.nanosec * 1e-9
        return sec if sec > 0 else None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


def main():
    p = argparse.ArgumentParser(description="Autoware fekezes-analizator")
    p.add_argument("--accel-topic", default="/localization/acceleration")
    p.add_argument("--velocity-topic", default="/vehicle/status/velocity_status")
    p.add_argument("--control-topic", default="/control/command/control_cmd")
    p.add_argument("--threshold", type=float, default=-0.5,
                   help="fekezes-kuszob m/s^2-ben (default: -0.5)")
    p.add_argument("--min-duration", type=float, default=0.2,
                   help="ennel rovidebb epizodokat eldob (s)")
    p.add_argument("--live", action="store_true", help="minden minta kiirasa")
    p.add_argument("--csv", default=None, help="mintak mentese CSV fajlba")
    args = p.parse_args()

    rclpy.init()
    node = BrakeMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.in_episode:
            node.finish_episode()
        print(f"\nOsszesen {node.episode_count} fekezesi epizod.", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()