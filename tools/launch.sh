#!/bin/bash

# A ROS_DOMAIN_ID lekérdezése az aktuális sessionből. Ha nincs, az alapértelmezés 1.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
N=$ROS_DOMAIN_ID

pip install paho-mqtt

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

# dynamic environment variables for Vanetza NAP

export VANETZA_INTERFACE=eth0
export VANETZA_STATION_ID="$N"
export VANETZA_STATION_TYPE=5
export VANETZA_MAC_ADDRESS=$(printf '6e:06:e0:03:00:%02x' "$N")


source /opt/autoware/setup.bash
source /home/aw/dev/install/setup.bash

ros2 launch fraternity_v2x cam.launch.py &
sleep 1
socktap -c /home/aw/tools/config.ini