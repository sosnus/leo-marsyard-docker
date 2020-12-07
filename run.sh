#!/bin/bash

# NOTE: change volume path, >>>> -v /<host pwd>:/data <<<<

# run interactive, detashed container
docker run -d -it --name ros -v /data/dockervolumes/ros:/data emulatorbot

# connect to shell in container
docker exec -it ros bash
