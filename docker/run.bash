#!/bin/bash
# Use: ./run.bash [vnc]

DC="dc.yml"
GPU_OVERRIDE=""

if [ "$1" == "vnc" ]; then
  echo "Using VNC"
  DC="dc_vnc.yml"
elif [ "$1" == "cpu" ]; then
  echo "Using X11 and CPU"
  # force use of cpu (ignore autodetect)
# Check for NVIDIA GPU and add GPU compose override if available
elif nvidia-detector 2> /dev/null; then
  echo "Autodetect nvidia drivers"
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
