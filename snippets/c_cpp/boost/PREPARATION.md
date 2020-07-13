# Note: Boost1.66 needs CMake 3.11
https://stackoverflow.com/questions/42123509/cmake-finds-boost-but-the-imported-targets-not-available-for-boost-version

## To install CMake 3.11
https://stackoverflow.com/questions/49859457/how-to-reinstall-the-latest-cmake-version

```
wget https://github.com/Kitware/CMake/releases/download/v3.11.0/cmake-3.11.0.tar.gz
tar zxvf cmake-3.11.0.tar.gz
cd cmake-3.11.0
sudo ./bootstrap
sudo make
sudo make install
```

## To install Boost1.66
https://stackoverflow.com/questions/8430332/uninstall-boost-and-install-another-version
https://stackoverflow.com/questions/3016448/how-can-i-get-cmake-to-find-my-alternative-boost-installation
```
sudo apt-get update
sudo apt-get -y --purge remove libboost-all-dev libboost-doc libboost-dev
sudo rm -f /usr/local/lib/libboost_*
sudo rm -f /usr/local/include/boost
sudo apt-get -y install build-essential g++ python-dev autotools-dev libicu-dev libbz2-dev
cd
wget http://downloads.sourceforge.net/project/boost/boost/1.66.0/boost_1_66_0.tar.gz
tar -zxvf boost_1_66_0.tar.gz
cd boost_1_66_0
#cpuCores=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`
#echo "Available CPU cores: "$cpuCores
cpuCores=2
# this will generate ./b2
./bootstrap.sh
sudo ./b2 --with=all -j $cpuCores install
```

Then, set the following variables accordingly.
```
set( BOOST_ROOT "/usr/local" CACHE PATH "Boost library path" )
set( Boost_NO_BOOST_CMAKE on CACHE BOOL "Do not use CMake for Boost(?)" )
set( Boost_NO_SYSTEM_PATHS on CACHE BOOL "Do not search system for Boost" )
```
