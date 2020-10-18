# practice to use a library to read and write xlsx files.

Using XLNT library
https://github.com/tfussell/xlnt

# How to install XLNT

```
git clone https://github.com/tfussell/xlnt.git
cd xlnt
mkdir build
cd build
cmake ..
make
sudo make install
```

# How to use XLNT
At the beginning of CMakeLists.txt,
```
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_library(xlnt_path NAMES xlnt)
message(STATUS ${xlnt_path})
add_library(xlnt SHARED IMPORTED)
set_target_properties(xlnt PROPERTIES IMPORTED_LOCATION ${xlnt_path})
```
so cmake can find the shared library.

Then, it can be followed by like this:
```
add_executable(example example.cpp)
target_link_libraries(example xlnt)
```
