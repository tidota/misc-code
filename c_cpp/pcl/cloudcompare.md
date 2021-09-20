# Evaluation with CloudCompare

CloudCompare needs a plugin to load a PCD file (qPCL).
Apparently, the default package does not have this plugin, and
it must be installed from the source.

Installation Instructions: https://github.com/cloudcompare/cloudcompare/blob/master/BUILD.md

# Installation notes

**On Ubuntu 20, there seems a problem on installation:** The snap may be the best.
https://snapcraft.io/install/cloudcompare/ubuntu

```
sudo apt update
sudo apt install snapd
sudo snap install cloudcompare
```

# Manual Installation (not successful on Ubuntu 20.04)

Install the prerequisites.
```
sudo apt update && sudo apt install libqt5svg5-dev libqt5opengl5-dev qt5-default qttools5-dev qttools5-dev-tools libqt5websockets5-dev
```

Clone the repository.
```
git clone --recursive https://github.com/cloudcompare/CloudCompare.git
cd CloudCompare
```

Configuration.
```
mkdir build && cd build
cmake --DPLUGIN_STANDARD_QPCL=ON ..
```

Build.
```
cmake --build .
```

Installation.
```
sudo cmake --install .
```

# Error message? `libCCAppCommon.so: cannot open shared object file`

It seems like there is a problem on Ubuntu 20.
It tries to install files in `/usr/local/bin`, `/usr/local/lib` and `/usr/local/share`,
but some files may be moved to a wrong directory?
