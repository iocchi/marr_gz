#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

NOCACHE=""
if [ "$1" == "no-cache" ]; then
  NOCACHE="--no-cache"
fi

docker build $NOCACHE -t marr_gz:system -f Dockerfile.system . && \
docker build $NOCACHE $UPAR -t marr_gz -f Dockerfile.user .

