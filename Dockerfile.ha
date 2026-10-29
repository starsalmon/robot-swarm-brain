FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-paho-mqtt \
    && rm -rf /var/lib/apt/lists/*
