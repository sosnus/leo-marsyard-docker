#!/bin/bash

build() {
  echo "Building EmulatorBot"
  docker build -f Dockerfile --tag emulatorbot --no-cache .
}

build
