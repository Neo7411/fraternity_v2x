#!/bin/bash

if [ -z "$1" ]; then
    echo "Error: container name required."
    echo "Usage: $0 <name> example: autoware_x"
    exit 1
fi

NAME_ARG="$1"
NUMB="${NAME_ARG##*_}"  


# a lenti container launch command ba ki klel cserelni a host file path eket 
#     --volume "$HOME/fraternity/VANETZA/fraternity_v2x/tools/:/home/aw/tools" \
rocker --x11 --privileged --nvidia \
    --network=vanetzalan0  \
    --volume "$HOME/fraternity/VANETZA/fraternity_v2x/v2x_modul/:/home/aw/dev" \
    --volume "$HOME/fraternity/VANETZA/fraternity_v2x/tools/:/home/aw/tools" \
    --volume "$HOME/autoware_maps/:/home/aw/maps" \
    --env AWID="$NUMB" \
    --env ROS_DOMAIN_ID="$NUMB" \
    --name "$NAME_ARG" \
    --hostname "$NAME_ARG" \
    -- autoware-vanetza:latest bash 