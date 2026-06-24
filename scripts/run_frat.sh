#!/bin/bash
# Egy jarmu (= egy konteneren) inditasa a kozos v2x_net halozaton.
# Hasznalat: ./run.sh <nev> pl. ./run.sh autoware_3
# A nev vegerol levett szam lesz az AWID (egyedi station_id + ROS_DOMAIN_ID).
# Tobb jarmuhoz inditsd tobbszor kulonbozo nevvel:
# ./run.sh autoware_1
# ./run.sh autoware_2

set -e

if [ -z "$1" ]; then
echo "Error: container name required."
echo "Usage: $0 <name> example: $0 autoware_3"
exit 1
fi

NAME_ARG="$1"
NUMB="${NAME_ARG##*_}" # a nev vegerol a szam (pl. autoware_3 -> 3)

# Kozos V2X bridge halozat letrehozasa, ha meg nincs. Ezen a bridge-en a
# szort (broadcast) CAM keretek eljutnak a tobbi jarmu konteneréhez.
docker network inspect v2x_net >/dev/null 2>&1 || docker network create v2x_net

xhost +local:root

docker run -it --rm \
--name "fraternity_sim_${NUMB}" \
--hostname "${NAME_ARG}" \
--network v2x_net \
--cap-add=NET_RAW \
--cap-add=NET_ADMIN \
--ulimit memlock=-1 \
--ulimit stack=67108864 \
-e DISPLAY=$DISPLAY \
-e AWID="$NUMB" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v ./autoware_maps:/workspace/autoware_map \
-v ./scripts/my_common.param.yaml:/opt/autoware/share/autoware_launch/config/planning/scenario_planning/common/common.param.yaml \
172.22.232.46:5000/fraternity_module:0.0.2