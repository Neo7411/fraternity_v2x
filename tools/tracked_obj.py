import rclpy
from rclpy.node import Node
import zenoh
import json
import math
import time

# ROS 2 és Autoware üzenettípusok
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from unique_identifier_msgs.msg import UUID
from autoware_perception_msgs.msg import TrackedObjects, TrackedObject, TrackedObjectKinematics
from autoware_perception_msgs.msg import ObjectClassification, Shape

def yaw_to_quaternion(yaw_rad):
    """Egyszerű konverzió Yaw (Z-tengely körüli forgás) szögből Quaternionba."""
    half_yaw = yaw_rad * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q

class ZenohCamToTracked(Node):
    def __init__(self):
        super().__init__('zenoh_cam_to_tracked_node')
        
        # 1. ROS 2 Publisher beállítása
        self.tracking_topic_name = '/perception/object_recognition/tracking/objects'
        self.track_pub = self.create_publisher(TrackedObjects, self.tracking_topic_name, 10)
        
        # 2. Zenoh konfiguráció és csatlakozás
        self.get_logger().info("🔌 Csatlakozás a Zenoh hálózathoz...")
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
        self.zenoh_session = zenoh.open(conf)
        
        # 3. Zenoh Subscriber beállítása
        self.zenoh_topic = "vanetza/out/cam"
        self.zenoh_sub = self.zenoh_session.declare_subscriber(self.zenoh_topic, self.zenoh_callback)
        self.get_logger().info(f"📡 Figyelem a '{self.zenoh_topic}' témát a Zenoh-n, és publikálom ROS 2-be!")

    def zenoh_callback(self, sample):
        """Ez hívódik meg, amikor bejön egy Zenoh (CAM) üzenet."""
        try:
            # Payload dekódolása
            try:
                payload_str = sample.payload.to_string()
            except AttributeError:
                payload_str = bytes(sample.payload).decode('utf-8')
            
            json_data = json.loads(payload_str)
            self.process_cam_to_ros(json_data)
            
        except json.JSONDecodeError:
            self.get_logger().warn("⚠️ Nem érvényes JSON érkezett!")
        except Exception as e:
            self.get_logger().error(f"❌ Hiba a feldolgozás során: {e}")

    def process_cam_to_ros(self, json_data):
        """A JSON adatok kinyerése és Autoware TrackedObjects üzenetté alakítása."""
        
        # 1. Alapadatok és biztonságos navigálás a JSON-ben (.get() használatával)
        station_id = json_data.get("stationID", 0)
        timestamp_sec = json_data.get("timestamp", time.time())
        
        fields = json_data.get("fields", {})
        cam = fields.get("cam", {}).get("camParameters", {})
        basic_container = cam.get("basicContainer", {})
        high_freq_container = cam.get("highFrequencyContainer", {}).get("basicVehicleContainerHighFrequency", {})
        
        ref_position = basic_container.get("referencePosition", {})
        
        # Koordináták kinyerése (CAM 0.1 mikro-fokokban adja meg, ezért osztás 10^7-nel)
        lat = ref_position.get("latitude", 0.0) / 10000000.0
        lon = ref_position.get("longitude", 0.0) / 10000000.0
        alt = ref_position.get("altitude", {}).get("altitudeValue", 0.0) / 100.0
        
        # Kinematika: sebesség, gyorsulás, irányszög
        heading_val = high_freq_container.get("heading", {}).get("headingValue", 0.0) / 10.0 # fok
        speed_val = high_freq_container.get("speed", {}).get("speedValue", 0.0) / 100.0 # m/s
        accel_val = high_freq_container.get("longitudinalAcceleration", {}).get("value", 0.0) / 10.0 # m/s^2
        
        # Méretek
        length_val = high_freq_container.get("vehicleLength", {}).get("vehicleLengthValue", 40) / 10.0 # m
        width_val = high_freq_container.get("vehicleWidth", 20) / 10.0 # m

        # ---------------------------------------------------------
        # 2. ROS 2 Üzenet felépítése
        # ---------------------------------------------------------
        tracked_msg = TrackedObjects()
        
        # Header kitöltése
        tracked_msg.header = Header()
        tracked_msg.header.frame_id = "map" # Cseréld ki az Autoware-ben használt frame-re
        tracked_msg.header.stamp.sec = int(timestamp_sec)
        tracked_msg.header.stamp.nanosec = int((timestamp_sec - int(timestamp_sec)) * 1e9)
        
        # Új TrackedObject létrehozása
        track_obj = TrackedObject()
        
        # A) Object ID (UUID generálása a stationID-ból)
        track_obj.object_id = UUID()
        # Átalakítjuk a stationID-t 16 bájtos tömbbé
        track_obj.object_id.uuid = list(station_id.to_bytes(16, byteorder='big'))
        
        # B) Osztályozás (Autó)
        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 1.0
        track_obj.classification.append(classification)
        
        # C) Alak és méretek (Bounding Box)
        track_obj.shape.type = Shape.BOUNDING_BOX
        track_obj.shape.dimensions.x = float(length_val)
        track_obj.shape.dimensions.y = float(width_val)
        track_obj.shape.dimensions.z = 1.5 # Alapértelmezett magasság, mert a CAM ritkán küldi
        
        # D) Kinematika beállítása
        track_kin = TrackedObjectKinematics()
        
        # - Pozíció (FIGYELEM: Ide Lat/Lon konverzió kell majd!)
        track_kin.pose_with_covariance.pose.position.x = float(lat) 
        track_kin.pose_with_covariance.pose.position.y = float(lon)
        track_kin.pose_with_covariance.pose.position.z = float(alt)
        
        # - Orientáció (Fokból Radiánba, majd Quaternion)
        heading_rad = math.radians(heading_val)
        track_kin.pose_with_covariance.pose.orientation = yaw_to_quaternion(heading_rad)
        
        # - Sebesség
        track_kin.twist_with_covariance.twist.linear.x = float(speed_val)
        track_kin.twist_with_covariance.twist.linear.y = 0.0
        track_kin.twist_with_covariance.twist.linear.z = 0.0
        
        # - Gyorsulás
        track_kin.acceleration_with_covariance.accel.linear.x = float(accel_val)
        
        # Kinematika hozzárendelése
        track_obj.kinematics = track_kin
        
        # E) Objektum hozzáadása a listához és publikálás
        tracked_msg.objects.append(track_obj)
        self.track_pub.publish(tracked_msg)
        
        self.get_logger().info(f"✅ V2X jármű publikálva! StationID: {station_id} | Sebesség: {speed_val} m/s")

    def destroy_node(self):
        """Kapcsolatok biztonságos lezárása kilépéskor."""
        self.get_logger().info("Leállítás: Zenoh kapcsolat lezárása...")
        self.zenoh_session.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZenohCamToTracked()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()