import zenoh
import json
import time

def listener(sample):
    try:
        # Payload dekódolása
        try:
            payload_str = sample.payload.to_string()
        except AttributeError:
            payload_str = bytes(sample.payload).decode('utf-8')
        
        json_data = json.loads(payload_str)
        station_id = json_data.get("stationID", "Ismeretlen")
        
        # Elnavigálunk a referencePosition részhez a JSON-ben
        cam_params = json_data.get("fields", {}).get("cam", {}).get("camParameters", {})
        ref_pos = cam_params.get("basicContainer", {}).get("referencePosition", {})
        
        # Nyers értékek kinyerése
        raw_lat = ref_pos.get("latitude", 0.0)
        raw_lon = ref_pos.get("longitude", 0.0)
        
        # ETSI Szabvány szűrése ("Unavailable" értékek kezelése)
        # 900000001 = Nincs GPS adat (Latitude)
        # 1800000001 = Nincs GPS adat (Longitude)
        if raw_lat >= 900000001 or raw_lon >= 1800000001:
            print(f"🚗 Jármű [{station_id}]: ⚠️ NINCS ÉRVÉNYES GPS ADAT (Álló/Beltéri helyzet)")
        else:
            # Osztás 10^7-nel, hogy megkapjuk a valós fokokat
            lat = raw_lat / 10000000.0
            lon = raw_lon / 10000000.0
            print(f"🚗 Jármű [{station_id}]: 🌍 Lat: {lat:.6f}, Lon: {lon:.6f}")
            
    except Exception as e:
        print(f"Hiba a feldolgozáskor: {e}")

if __name__ == "__main__":
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    print("🔌 Kapcsolódás a Zenoh-hoz... (tcp/127.0.0.1:7447)")
    session = zenoh.open(conf)
    
    topic = "vanetza/out/cam"
    sub = session.declare_subscriber(topic, listener)
    
    print(f"📍 Figyelem a koordinátákat a '{topic}' témán...\n" + "-"*50)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nKilépés...")
    finally:
        session.close()