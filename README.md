# Leo Docker #


### Usage ###

Run command below with leo-vanilla repository credentials to build docker image:
```bash
$ sudo ./build.sh 
```

Next, you can run container with:
```bash
$ sudo docker run -it --name=<CONTAINER_NAME> --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" marsyard_leo:latest
```

Enable X server on host:
```sh
$ xhost +local:root
```

or if you're concerned about security:
```sh
$ xhost +local:`docker inspect --format='{{ .Config.Hostname }}' <CONTAINER_NAME>`
```

Then, you can run command in running container:
```sh
$ sudo docker exec -it <CONTAINER_NAME>> /bin/bash
```
