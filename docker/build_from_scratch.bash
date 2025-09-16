#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

NOCACHE=""  # --no-cache"

docker pull ros:jazzy-ros-base-noble

cd ..

docker build $NOCACHE -t marr_gz:system -f docker/Dockerfile.system . && \
docker build $NOCACHE $UPAR -t marr_gz -f docker/Dockerfile.user .

docker tag marr_gz:system iocchi/marr_gz:system 

