#!/bin/bash

if [ -z "$1" ]; then
    echo "Error: container name required."
    echo "Usage: $0 <name> example: autoware_x"
    exit 1
fi

NAME_ARG="$1"
NUMB="${NAME_ARG##*_}"  


# a lenti s

rocker --x11 --privileged --nvidia \
    --network=v2x_net  \
    --volume "$HOME/fraternity/VANETZA/fraternity_v2x/tools/:/home/aw/tools" \
    --volume "$HOME/fraternity/VANETZA/fraternity_v2x/v2x_modul/:/home/aw/dev" \
    --volume "$HOME/fraternity/VANETZA/fraternity_v2x/vanetza/:/home/aw/vanetza_src" \
    --volume "$HOME/autoware_maps/:/home/aw/maps" \
    --name "$NAME_ARG" \
    --hostname "$NAME_ARG" \
    -- ghcr.io/autowarefoundation/autoware:universe-devel-cuda-humble bash