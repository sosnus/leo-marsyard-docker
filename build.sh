#!/bin/bash

build() {
  echo "Building Leo"
  docker build -f Dockerfile.leo --tag marsyard_leo_2 --no-cache .
}

build
