#!/bin/bash

# Use: ./run.bash [vnc]
# Autodetect nvidia drivers and run nvidia runtime

DOCKER_RUNTIME="runc"

if nvidia-detector 2> /dev/null; then
  NVIDIA_DETECT=`nvidia-detector`
  if [ "$NVIDIA_DETECT" != "None"  ]; then
    DOCKER_RUNTIME="nvidia"
    echo "Nvidia detect: ${NVIDIA_DETECT} Using nvidia runtime !!!"
  fi
fi

DC="dc.yml"

if [ "$1" == "vnc" ]; then
  DC="dc_vnc.yml"
fi

docker rm marr_gz -fv

# --force-recreate
DOCKER_RUNTIME=${DOCKER_RUNTIME} docker compose  -f $DC up -d  --remove-orphans -V && \
sleep 3 && \
docker exec -it marr_gz tmux a

