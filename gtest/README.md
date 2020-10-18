# Google Test practice

To install Google test, run the following commands.
```
sudo apt-get install libgtest-dev cmake
cd /usr/src/gtest
sudo cmake .
sudo make

# copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
sudo cp *.a /usr/lib
```

To run the practice code,
```
mkdir build
cd build
cmake ..
make
./UNIT-test1
```

The output will look like this.
```
[==========] Running 3 tests from 2 test cases.
[----------] Global test environment set-up.
[----------] 2 tests from initZero
[ RUN      ] initZero.OneToZero
[       OK ] initZero.OneToZero (0 ms)
[ RUN      ] initZero.ThreeToZero
[       OK ] initZero.ThreeToZero (0 ms)
[----------] 2 tests from initZero (0 ms total)

[----------] 1 test from MyTest
[ RUN      ] MyTest.SomeTest
[       OK ] MyTest.SomeTest (0 ms)
[----------] 1 test from MyTest (0 ms total)

[----------] Global test environment tear-down
[==========] 3 tests from 2 test cases ran. (1 ms total)
[  PASSED  ] 3 tests.
```

