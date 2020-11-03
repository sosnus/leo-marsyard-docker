 #!/bin/bash

# NOTE: change volume path, >>>> -v /<host pwd>:/data <<<<

# run interactive, detashed container
docker run -d -it --name rosbag -v /data/dockervolumes/ros:/data rosbag_sim

# connect to shell in container
docker exec -it rosbag bash
