import zenoh
import time
import json

# Callback függvény az üzenetek feldolgozásához
def listener(sample):
    try:
        # 1. Payload dekódolása a kompatibilitási logika alapján
        try:
            payload_str = sample.payload.to_string()
        except AttributeError:
            payload_str = bytes(sample.payload).decode('utf-8')
        
        # 2. JSON beolvasása
        json_data = json.loads(payload_str)
        
        # 3. Adatok kinyerése és kiírása (pl. stationID)
        station_id = json_data.get("stationID", "Nincs megadva")
        print(f"📩 Új üzenet a '{sample.key_expr}' témán | stationID: {station_id}")
        
        # Opcionális: Ha az egész JSON-t látni akarod szépen formázva, 
        # vedd ki a kommentet az alábbi sor elől:
        # print(json.dumps(json_data, indent=2, ensure_ascii=False))

    except json.JSONDecodeError:
        print(f"⚠️ Nem JSON formátumú adat érkezett: {payload_str}")
    except Exception as e:
        print(f"❌ Hiba a feldolgozás során: {e}")

if __name__ == "__main__":
    # 1. Konfiguráció beállítása a működő példád alapján
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7447"]')
    
    # 2. Csatlakozás a megadott beállításokkal
    print("🔌 Csatlakozás a Zenoh hálózathoz (tcp/127.0.0.1:7447)...")
    session = zenoh.open(conf)

    # 3. Key Expression (Téma) meghatározása
    # FONTOS: Nincs kezdő perjel!
    key_expr = "vanetza/out/cam" 
    # Ha minden vanetza üzenetet akarsz: key_expr = "vanetza/**"

    print(f"📡 Várakozás a JSON üzenetekre a következő témán: '{key_expr}'...")
    print("🛑 A leállításhoz nyomd meg a Ctrl+C gombot.\n")
    print("-" * 50)

    # 4. Feliratkozás
    sub = session.declare_subscriber(key_expr, listener)

    # 5. Fő szál futásban tartása
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🚪 Kilépés...")
    finally:
        session.close()