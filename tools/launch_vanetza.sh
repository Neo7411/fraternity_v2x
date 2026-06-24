#!/bin/bash

# Az N értékének beállítása (első argumentum, alapértelmezés: 1)
N=${1:-1}

echo "Konfiguráció inicializálása (N=$N)..."

# Hostnév felvétele 
echo "127.0.0.1 $(hostname)" >> /etc/hosts

# --- MQTT BROKER ELLENŐRZÉSE ÉS INDÍTÁSA ---
if pgrep -x "mosquitto" > /dev/null
then
    echo "[OK] A Mosquitto broker már fut a háttérben."
else
    echo "[INFO] A Mosquitto broker nem fut. Indítás..."
    mosquitto -d
    sleep 1 # Várjunk 1 másodpercet, hogy biztosan felálljon a szerver
fi

# Környezeti változók beállítása
export VANETZA_INTERFACE=eth0
export VANETZA_STATION_ID="$N"
export VANETZA_STATION_TYPE=5
export VANETZA_MAC_ADDRESS=$(printf '6e:06:e0:03:00:%02x' "$N")
export VANETZA_IGNORE_OWN_MESSAGES=true
export VANETZA_RSSI_ENABLED=false
export VANETZA_USE_HARDCODED_GPS=true
export VANETZA_LATITUDE=$(python3 -c "print(47.5316 + $N*0.0005)")
export VANETZA_LONGITUDE=21.6273

# --- MQTT BEÁLLÍTÁSOK ---
export VANETZA_CAM_MQTT_ENABLED=true
export VANETZA_CAM_ZENOH_ENABLED=false
export VANETZA_CAM_PERIODICITY=0
export VANETZA_LOCAL_MQTT_BROKER="127.0.0.1"

echo "Környezeti változók beállítva. (MAC: $VANETZA_MAC_ADDRESS, LAT: $VANETZA_LATITUDE)"

# Socktap indítása
echo "Socktap indítása..."
socktap -c /home/aw/tools/config.ini    