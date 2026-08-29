#!/bin/bash

# A ROS_DOMAIN_ID lekérdezése az aktuális sessionből. Ha nincs, az alapértelmezés 1.
# FIGYELEM: a `sudo su` alapból kitörli a környezetet (sudoers env_reset), így
# rootként a docker -e ROS_DOMAIN_ID elveszik, és MINDKÉT konténer 1-et kapna
# -> azonos station ID -> nem látják egymást. Használd: sudo -E su
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "FIGYELEM: nincs ROS_DOMAIN_ID a környezetben, 1 lesz belőle."
    echo "          Ha 'sudo su'-val váltottál rootot, lépj ki és használd: sudo -E su"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
N=$ROS_DOMAIN_ID

pip install paho-mqtt

echo "Konfiguráció inicializálása (N=$N, ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."

# Hostnév felvétele 
echo "127.0.0.1 $(hostname)" >> /etc/hosts

# --- MQTT BROKER ELLENŐRZÉSE ÉS INDÍTÁSA ---
if pgrep -x "mosquitto" > /dev/null
then
    echo "{OK} A Mosquitto broker már fut a háttérben."
else
    echo "{INFO} A Mosquitto broker nem fut. Indítás..."
    mosquitto -d
    sleep 1 # Várjunk 1 másodpercet, hogy biztosan felálljon a szerver
fi

# Környezeti változók beállítása
export VANETZA_INTERFACE=eth0
export VANETZA_STATION_ID="$N"
export VANETZA_STATION_TYPE=5
export VANETZA_MAC_ADDRESS=$(printf '6e:06:e0:03:00:%02x' "$N")

echo "Launching Vanetza NAP (station id=$N, mac=$VANETZA_MAC_ADDRESS)..."
socktap -c /home/aw/tools/config.ini