#!/bin/bash

# Az N értékének beállítása (első argumentum, alapértelmezés: 1)
N=${1:-1}

echo "Konfiguráció inicializálása (N=$N)..."

# Hostnév felvétele (Megjegyzés: ehhez a lépéshez root/rendszergazdai jog kellhet)
echo "127.0.0.1 $(hostname)" >> /etc/hosts

# Zenoh konfiguráció módosítása
sed -i 's/"enabled": true/"enabled": false/' /zenoh_config.json5

export VANETZA_INTERFACE=eth0
export VANETZA_STATION_ID="$N"
export VANETZA_STATION_TYPE=5
export VANETZA_MAC_ADDRESS=$(printf '6e:06:e0:03:00:%02x' "$N")
export VANETZA_IGNORE_OWN_MESSAGES=true
export VANETZA_RSSI_ENABLED=false
export VANETZA_USE_HARDCODED_GPS=true
export VANETZA_LATITUDE=$(python3 -c "print(47.5316 + $N*0.0005)")
export VANETZA_LONGITUDE=21.6273
export VANETZA_ZENOH_LOCAL_ONLY=true
export VANETZA_CAM_ZENOH_ENABLED=true
export VANETZA_CAM_MQTT_ENABLED=false
export VANETZA_CAM_PERIODICITY=0
echo "Környezeti változók beállítva. (MAC: $VANETZA_MAC_ADDRESS, LAT: $VANETZA_LATITUDE)"

# Socktap indítása (sudo nélkül, ahogy kérted)
echo "Socktap indítása..."
socktap -c /home/aw/tools/config.ini