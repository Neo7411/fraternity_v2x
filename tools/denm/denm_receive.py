#!/usr/bin/env python3
import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from tier4_external_api_msgs.srv import SetEmergency


class EeblBrake(Node):
    def __init__(self):
        super().__init__("eebl_brake")

        self.cli = self.create_client(SetEmergency, "/api/autoware/set/emergency")
        if not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/api/autoware/set/emergency not available yet")

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt.on_connect = lambda c, u, f, rc, p: c.subscribe("vanetza/out/denm")
        self.mqtt.on_message = self.on_message
        self.mqtt.connect("localhost", 1883)
        self.mqtt.loop_start()

        self.get_logger().info("Waiting for EEBL DENM on vanetza/out/denm ...")

    def on_message(self, client, userdata, msg):
        try:
            denm = json.loads(msg.payload)
        except ValueError:
            return

        et = (denm.get("fields", {}).get("denm", {})
                  .get("situation", {}).get("eventType", {}))
        is_eebl = any(k.startswith("dangerousSituation") for k in et.get("ccAndScc", {})) \
            or et.get("causeCode") == 99
        if not is_eebl:
            return

        self.get_logger().warn("EEBL received -> EMERGENCY STOP")
        req = SetEmergency.Request()
        req.emergency = True
        self.cli.call_async(req)


def main():
    rclpy.init()
    node = EeblBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()