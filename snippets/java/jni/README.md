# JNI practice

This directory includes C/C++ and java code to practice JNI functionality.

1. Write Java code `User.java` which will call native code.

1. Compile the Java code.

  ```
  javac User.java -h .
  ```

1. Write C/C++ code based on the header file (+ Cmake)

1. Compile the C/C++ code.

  ```
  mkdir build
  cd build
  cmake ..
  make
  cd ..
  ```

1. Run the code

  ```
  java -Djava.library.path=./build User
  ```
