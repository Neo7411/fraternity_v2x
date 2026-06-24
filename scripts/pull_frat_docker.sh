#!/bin/bash

VERSION="0.0.2"
REGISTRY="172.22.232.46:5000"
IMAGE_NAME="fraternity_module"

echo "Letöltés (pull) megkezdése: ${REGISTRY}/${IMAGE_NAME}:${VERSION} ..."
docker pull ${REGISTRY}/${IMAGE_NAME}:${VERSION}