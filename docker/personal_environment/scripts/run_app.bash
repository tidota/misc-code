#!/bin/bash
container=main_container
if [[ $# -eq 0 ]]; then
docker exec -it $container bash
else
docker exec $container $1
fi

