# OpenCV

OpenCV is a set of libraries for image processing.


# Insallation

```
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

```
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout <version>
mkdir build
cmake ..
make -j4
sudo make install
```
