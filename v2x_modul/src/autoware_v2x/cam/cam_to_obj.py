import rclpy
from rclpy.node import Node

# Import both message types and their internal object types
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrackedObjects, TrackedObject, TrackedObjectKinematics

class CAMToTracked(Node):
    
    def __init__(self):
        super().__init__('cam_to_tracked_node')
        
        # 1. Topics
        self.cam_topic_name = '/v2x/cam/objects'
        self.tracking_topic_name = '/perception/object_recognition/tracking/objects'
        
        # 2. Subscriber (Receives PredictedObjects from V2X)
        self.cam_sub = self.create_subscription(
            PredictedObjects, 
            self.cam_topic_name, 
            self.cam_cb, 
            10
        )
        
        # 3. Publisher (Sends TrackedObjects to Autoware)
        self.track_pub = self.create_publisher(
            TrackedObjects, 
            self.tracking_topic_name, 
            10
        )

    def cam_cb(self, msg):
        # Create a new empty TrackedObjects message
        tracked_msg = TrackedObjects()
        tracked_msg.header = msg.header
        
        # Loop through the incoming PredictedObjects
        for pred_obj in msg.objects:
            
            # Extract the integer Vehicle ID for logging
            raw_uuid_bytes = bytes(pred_obj.object_id.uuid)
            vehicle_id = int.from_bytes(raw_uuid_bytes[0:4], byteorder='big')
            self.get_logger().debug(f"Converting and forwarding Vehicle ID: {vehicle_id}")
            
            # Create a new TrackedObject
            track_obj = TrackedObject()
            
            # Map standard fields
            track_obj.object_id = pred_obj.object_id
            track_obj.classification = pred_obj.classification
            track_obj.shape = pred_obj.shape
            
            # --- THE FIX: Map Kinematics Manually ---
            track_kin = TrackedObjectKinematics()
            
            # Map 'initial_pose' to 'pose', 'initial_twist' to 'twist', etc.
            track_kin.pose_with_covariance = pred_obj.kinematics.initial_pose_with_covariance
            track_kin.twist_with_covariance = pred_obj.kinematics.initial_twist_with_covariance
            track_kin.acceleration_with_covariance = pred_obj.kinematics.initial_acceleration_with_covariance
            
            # Assign the newly built kinematics object back to the tracked object
            track_obj.kinematics = track_kin
            # ----------------------------------------

            # Append the newly constructed TrackedObject to our outgoing message array
            tracked_msg.objects.append(track_obj)
            
        # Publish the fully converted message
        self.track_pub.publish(tracked_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CAMToTracked()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()