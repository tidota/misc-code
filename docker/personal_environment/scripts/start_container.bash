#!/bin/bash
hostip=$(ipconfig.exe | grep 'vEthernet (WSL)' -A4 | cut -d":" -f 2 | tail -n1 | sed -e 's/\s*//g')
win_home="/mnt/c/Users/Tetsuya:/media/win_home"
ssh_dir="/home/tidota/.ssh:/home/developer/.ssh"
#image="tidota/ubuntu-personal"
image="ubuntu-personal:latest"
container=main_container
docker run -it --rm --name $container -v $win_home -v $ssh_dir -e DISPLAY=$hostip:0.0 $image

