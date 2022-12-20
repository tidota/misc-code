# Docker environment for ROS2

The version to target here is ROS2 foxy. Hence, Ubuntu 20.04 needs to be installed.

# Misc commands

Assuming Docker has been installed.
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
./build.bash ros2-foxy
```

# Pull a Docker image
Instead of building an image, you can pull a prepared one.

To pull images,
```
docker pull <ID>/<image name>
```
For example,
```
docker pull tidota/ros2-foxy
```

# Run a Docker image

Use `run.bash`

For example,
```
./run.bash ros2-foxy /home/tidota/catkin_ws
```
If running a pulled one,
```
./run.bash tidota:ros2-foxy /home/tidota/catkin_ws
```


In another terminal,
```
docker exec -it <some name> bash
```
