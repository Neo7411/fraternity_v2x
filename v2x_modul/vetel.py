#!/usr/bin/env python3
"""
vetel.py (cam_to_obj.py)
Vanetza Zenoh (RX) -> CAM JSON -> Autoware TrackedObjects (map frame)

Függőségek:
    pip install eclipse-zenoh pymap3d

Indítás:
    source /opt/autoware/setup.bash
    python3 vetel.py
"""

import json
import math
import rclpy
from rclpy.node import Node
import pymap3d as pm
import zenoh

from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion
# FIGYELEM: _auto_ kivéve, Shape hozzáadva!
from autoware_perception_msgs.msg import TrackedObjects, TrackedObject, TrackedObjectKinematics, ObjectClassification, Shape


def heading_to_quat(heading_deg):
    """ETSI heading (fok, Észak=0, CW) -> ENU quaternion (Kelet=0, CCW)."""
    # ETSI -> ENU yaw
    yaw_rad = math.radians(90.0 - heading_deg)
    
    # Yaw to Quaternion (z-tengely körüli forgatás)
    half_yaw = yaw_rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


class CamToObjZenoh(Node):
    def __init__(self):
        super().__init__("cam_to_obj")

        # --- Paraméterek ---
        self.declare_parameter("map_origin_lat", 47.53)
        self.declare_parameter("map_origin_lon", 21.62)
        self.declare_parameter("map_origin_alt", 120.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("my_station_id", 1)  # Saját magunkat kiszűrjük
        self.declare_parameter("zenoh_endpoint", "tcp/127.0.0.1:7447")
        self.declare_parameter("cam_rx_topic", "vanetza/out/cam")

        gp = self.get_parameter
        self.lat0 = gp("map_origin_lat").value
        self.lon0 = gp("map_origin_lon").value
        self.alt0 = gp("map_origin_alt").value
        self.map_frame = gp("map_frame").value
        self.my_station_id = int(gp("my_station_id").value)
        endpoint = gp("zenoh_endpoint").value
        self.cam_rx_topic = gp("cam_rx_topic").value

        # --- ROS 2 Publisher ---
        self.obj_pub = self.create_publisher(TrackedObjects, '/perception/object_recognition/tracking/objects', 10)

        # --- Zenoh Subscriber ---
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", json.dumps([endpoint]))
        self.zsession = zenoh.open(conf)
        
        # Feliratkozás a bejövő CAM üzenetekre
        self.sub = self.zsession.declare_subscriber(self.cam_rx_topic, self.on_cam_received)
        self.get_logger().info(f"Zenoh RX csatlakoztatva -> {endpoint}, kulcs: {self.cam_rx_topic}")

    def on_cam_received(self, sample):
        try:
            payload = sample.payload.decode('utf-8')
            cam_data = json.loads(payload)
            
            station_id = cam_data.get("stationId")
            
            # Ne detektáljuk saját magunkat "másik autóként"
            if station_id == self.my_station_id:
                return

            self.process_cam_to_object(station_id, cam_data)

        except Exception as e:
            self.get_logger().error(f"Hiba a CAM dekódolása során: {e}")

    def process_cam_to_object(self, station_id, cam_data):
        try:
            # Json parsálás 
            basic_container = cam_data["camParameters"]["basicContainer"]
            hf_container = cam_data["camParameters"]["highFrequencyContainer"]["basicVehicleContainerHighFrequency"]
            
            lat = basic_container["referencePosition"]["latitude"]
            lon = basic_container["referencePosition"]["longitude"]
            alt = basic_container["referencePosition"].get("altitude", {}).get("altitudeValue", 800001)
            
            # Ha nincs rendes magasság adat, használjuk a map origót
            if alt == 800001:
                alt = self.alt0

            heading = hf_container["heading"]["headingValue"]
            speed = hf_container["speed"]["speedValue"]

            # 1. WGS84 -> ENU (map frame) transzformáció
            east, north, up = pm.geodetic2enu(lat, lon, alt, self.lat0, self.lon0, self.alt0)

            # 2. TrackedObject összeállítása
            obj_msg = TrackedObjects()
            obj_msg.header.stamp = self.get_clock().now().to_msg()
            obj_msg.header.frame_id = self.map_frame

            t_obj = TrackedObject()
            
            # ObjectID generálás (StationID-ből)
            t_obj.object_id.uuid = station_id.to_bytes(16, byteorder='big')
            
            # Osztályozás (Vehicle)
            classification = ObjectClassification()
            classification.label = ObjectClassification.CAR
            classification.probability = 1.0
            t_obj.classification.append(classification)

            # Kinematika (Pozíció és Sebesség)
            kinematics = TrackedObjectKinematics()
            
            pose = Pose()
            pose.position.x = east
            pose.position.y = north
            pose.position.z = up
            pose.orientation = heading_to_quat(heading)
            kinematics.pose_with_covariance.pose = pose

            twist = Twist()
            # A sebesség vektor az aktuális heading irányába mutat
            yaw_rad = math.radians(90.0 - heading)
            twist.linear.x = speed * math.cos(yaw_rad)
            twist.linear.y = speed * math.sin(yaw_rad)
            kinematics.twist_with_covariance.twist = twist

            t_obj.kinematics = kinematics
            
            # Objektum méretek 
            t_obj.shape.type = Shape.BOUNDING_BOX
            t_obj.shape.dimensions = Vector3(x=4.9, y=1.9, z=1.5) 

            obj_msg.objects.append(t_obj)

            # 3. Publikálás Autoware felé
            self.obj_pub.publish(obj_msg)
            
            self.get_logger().info(
                f"Objektum generálva (Station {station_id}): X={east:.1f}, Y={north:.1f}, V={speed:.1f}m/s",
                throttle_duration_sec=2.0
            )

        except KeyError as e:
            self.get_logger().warn(f"Hiányzó kulcs a CAM üzenetben: {e}")

    def destroy_node(self):
        try:
            self.zsession.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CamToObjZenoh()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()