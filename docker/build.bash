#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

cd ..

docker pull iocchi/marr_gz:system
docker tag iocchi/marr_gz:system marr_gz:system
docker build $UPAR -t marr_gz -f docker/Dockerfile.user .

