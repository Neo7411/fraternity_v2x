#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import paho.mqtt.client as mqtt
import json 

# ---------------- BEÁLLÍTÁSOK ----------------
BROKER_ADDRESS = "127.0.0.1"  # A Mosquitto szerver IP-je
PORT = 1883                   # Alapértelmezett MQTT port
TOPIC = "vanetza/in/cam"      # Ide küldjük az adatot
# ---------------------------------------------


def on_connect(client, userdata, flags, reason_code, properties):
    """Visszahívó függvény (callback), ha sikeres a csatlakozás."""
    if reason_code == 0:
        print(f"Sikeresen csatlakozva a brokerhez: {BROKER_ADDRESS}:{PORT}")
    else:
        print(f"Sikertelen csatlakozás. Hibakód: {reason_code}")

def main():
    # Kliens létrehozása (a VERSION2 eltünteti a deprecation warningot)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect

    with open('./asd.json', 'r') as f:
        template = json.load(f)
        
    print(f"Csatlakozási kísérlet...")
    try:
        client.connect(BROKER_ADDRESS, PORT, 60)
    except ConnectionRefusedError:
        print(f"HIBA: A kapcsolat elutasítva! Fut a Mosquitto broker a {BROKER_ADDRESS} címen?")
        return

    # Elindítjuk az MQTT kliens hálózati szálát a háttérben
    client.loop_start()

    print(f"\nKészen áll a küldésre a '{TOPIC}' témára. (Kilépés: Ctrl+C)")
    print("-" * 50)

    try:
        counter = 1
        while True:
            
            # A szótárat (dictionary) visszaalakítjuk JSON stringgé
            payload_str = json.dumps(template)
            
            # Üzenet publikálása a string payload-al
            client.publish(TOPIC, payload_str)
            print(f"[KÜLDVE] -> {payload_str}")
            
            counter += 1
            time.sleep(2)  # 2 másodperc szünet
            
    except KeyboardInterrupt:
        print("\nLeállítás kérése...")
    finally:
        # Tisztességes lekapcsolódás
        client.loop_stop()
        client.disconnect()
        print("Kapcsolat lezárva.")

if __name__ == "__main__":
    main()