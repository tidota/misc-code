# Docker environment for ROS and Gazebo

You know what I want to do here, right?

# Docker Installation

Just referred to https://github.com/osrf/subt/wiki/Docker%20Install

```
sudo apt-get remove docker docker-engine docker.io
sudo apt install curl apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) edge"
sudo apt-get update && sudo apt-get install docker-ce docker-ce-cli
```

To test it,
```
sudo docker run hello-world
```

# Docker Setup

```
docker login
```
Then, log in by your ID and password

To list images,
```
docker images
```

# Build a Docker image
Create Dockerfile

well, I need to add more details here...

Finally, use `build.bash`
```
./build.bash ros-noetic
```

# Pull a Docker image
Instead of building an image, you can pull a prepared one.

To pull images,
```
docker pull <ID>/<image name>
```
For example,
```
docker pull tidota/ros-noetic
```

# Run a Docker image

Use `run.bash`

For example,
```
./run.bash ros-noetic /home/tidota/catkin_ws_p
```
If running a pulled one,
```
./run.bash tidota:ros-noetic /home/tidota/catkin_ws_p
```


In another terminal,
```
docker exec -it <some name> bash
```
