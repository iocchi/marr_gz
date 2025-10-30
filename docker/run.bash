#!/bin/bash
# Use: ./run.bash [vnc]

DC="dc.yml"
GPU_OVERRIDE=""

if [ "$1" == "vnc" ]; then
  DC="dc_vnc.yml"
fi

# Check for NVIDIA GPU and add GPU compose override if available
if nvidia-detector 2> /dev/null; then
  NVIDIA_DETECT=`nvidia-detector`
  if [ "$NVIDIA_DETECT" != "None"  ]; then
    echo "Nvidia detect: ${NVIDIA_DETECT} Using GPU support !!!"
    GPU_OVERRIDE="-f dc.gpu.yml"
  fi
fi

docker compose -f $DC $GPU_OVERRIDE up -d --remove-orphans -V && \
sleep 3 && \
docker exec -it marr_gz tmux a
docker compose -f $DC $GPU_OVERRIDE rm -f