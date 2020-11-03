#!/bin/bash

build() {
  echo "Building smart"
  docker build -f Dockerfile.leo --tag rosbag_sim --no-cache .
}

build
