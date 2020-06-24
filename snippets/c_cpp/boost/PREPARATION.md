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

Then, set the following variables accordingly.
```
set( BOOST_ROOT "/usr/local" CACHE PATH "Boost library path" )
set( Boost_NO_BOOST_CMAKE on CACHE BOOL "Do not use CMake for Boost(?)" )
set( Boost_NO_SYSTEM_PATHS on CACHE BOOL "Do not search system for Boost" )
```