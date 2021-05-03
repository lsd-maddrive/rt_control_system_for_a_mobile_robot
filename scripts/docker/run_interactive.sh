#!/bin/bash
sudo docker container run -e DISPLAY=$DISPLAY -it tracked_robot /bin/bash
sudo docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    tracked_robot
