#!/bin/bash
container=main_container
command=`echo $1 | awk '{split($0,a," "); print a[1]}'`
command=`echo $command | sed "s/\//_/g"`
if [[ $# -eq 0 ]]; then
docker exec -it $container bash
else
docker exec $container $1 > /mnt/c/Users/Tetsuya/Documents/management/docker_"$command"_log.txt
fi

