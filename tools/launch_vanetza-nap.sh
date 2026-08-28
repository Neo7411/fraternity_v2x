#!/bin/bash

# A ROS_DOMAIN_ID lekérdezése az aktuális sessionből. Ha nincs, az alapértelmezés 1.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
N=$ROS_DOMAIN_ID

pip install paho-mqtt

echo "Konfiguráció inicializálása (N=$N, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."

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
export VANETZA_LATITUDE=47.5316 
export VANETZA_LONGITUDE=21.6273

# --- MQTT BEÁLLÍTÁSOK ---
export VANETZA_CAM_MQTT_ENABLED=true
export VANETZA_CAM_ZENOH_ENABLED=false
export VANETZA_CAM_PERIODICITY=0
export VANETZA_LOCAL_MQTT_BROKER="127.0.0.1"

echo "Környezeti változók beállítva. (MAC: $VANETZA_MAC_ADDRESS, LAT: $VANETZA_LATITUDE)"

# # --- AUTOWARE KÖRNYEZET BETÖLTÉSE ---
# echo "Autoware környezet source-olása..."
# source /opt/autoware/setup.bash

# # # --- PYTHON SCRIPTEK INDÍTÁSA A HÁTTÉRBEN ---
# echo "V2X CAM send/receive scriptek indítása a háttérben..."
# python3 /home/aw/tools/cam/cam_send.py &
# python3 /home/aw/tools/cam/cam_receive.py &

# # Várunk egy picit, hogy a ROS 2 node-ok biztosan elinduljanak a socktap előtt
sleep 1 

# Socktap indítása az előtérben
echo "Socktap indítása..."
socktap -c /home/aw/tools/config.ini