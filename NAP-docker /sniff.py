#!/usr/bin/env python3
"""
zenoh_sniffer.py
Minden Zenoh forgalom lehallgatása a megadott routeren.
"""

import time
import json
import argparse
import zenoh

def listener(sample):
    print(f"\n--- Új üzenet érkezett ---")
    print(f"Kulcs (Topic): {sample.key_expr}")
    
    # Megpróbáljuk dekódolni az üzenetet szövegként/JSON-ként
    try:
        payload_str = sample.payload.decode('utf-8')
        try:
            # Ha JSON, akkor szépen formázva írjuk ki
            data = json.loads(payload_str)
            print("Tartalom (JSON):")
            print(json.dumps(data, indent=2))
        except json.JSONDecodeError:
            # Ha csak sima szöveg
            print(f"Tartalom (Szöveg): {payload_str}")
    except Exception:
        # Ha bináris adat (nem dekódolható szövegként)
        print(f"Tartalom (Bináris): {sample.payload}")

def main():
    parser = argparse.ArgumentParser(description="Zenoh Sniffer")
    parser.add_argument("-e", "--endpoint", type=str, default="tcp/127.0.0.1:7447",
                        help="A Zenoh router végpontja (pl. tcp/127.0.0.1:7447)")
    args = parser.parse_args()

    print(f"Csatlakozás a Zenoh routerhez: {args.endpoint}")
    
    # Zenoh konfiguráció beállítása
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    
    # Munkamenet nyitása
    session = zenoh.open(conf)
    
    # Feliratkozás MINDENRE (**)
    sub = session.declare_subscriber("**", listener)
    
    print("Sikeres csatlakozás! Várakozás az üzenetekre... (Kilépés: Ctrl+C)")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nKilépés...")
    finally:
        session.close()

if __name__ == "__main__":
    main()