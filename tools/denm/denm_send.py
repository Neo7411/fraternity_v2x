import paho.mqtt.client as mqtt
import json

STATION_ID = 175


def main():
    template_path = '/vanetza/examples/in_denm.json'

    with open(template_path, 'r') as f:
        denm_msg = json.load(f)

    # sender = originating station ID
    denm_msg["management"]["actionId"]["originatingStationId"] = STATION_ID

    # make it an EEBL: causeCode 99 (dangerousSituation),
    # subCause 1 = emergencyElectronicBrakeEngaged
    denm_msg["situation"]["eventType"] = {"ccAndScc": {"dangerousSituation99": 1}}

    print("-" * 60 + "\n" + json.dumps(denm_msg, indent=2) + "\n" + "-" * 60)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect("localhost", 1883)
    client.loop_start()

    print("Push message")
    info = client.publish("vanetza/in/denm", json.dumps(denm_msg))
    info.wait_for_publish()
    print("PUSHED")

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()