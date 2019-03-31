# JNI practice

This directory includes C/C++ and java code to practice JNI functionality.

`User.java` defines `Node` class which can construct a tree structure.
`database.cpp` provides functionalities to get actual data to build a tree (at
this point, the data is just dummy). `DBWrapper.cpp` provides a wrapper so the
database layer can be used by the java code.

As `database.cpp` reads data of each node from a file, it calls a callback
function given by the higher layer, i.e., `DBWrapper.cpp`. The called function
in `DBWrapper.cpp` then calls another callback function of `User.java`. This
callback function finally creates an actual object.

```
                     User.java
------------------------------------------------------------------------
            ↓ load            ↑ createNode (callback)
------------------------------------------------------------------------
                    DBWrapper.cpp
------------------------------------------------------------------------
            ↓ load            ↑ createNode (callback)
------------------------------------------------------------------------
            database.cpp (reads data from a file.)
```

The definition of node is invisible to `database.cpp` and the file format is
invisible to `User.java`. The only interface between these layers is `Params`
class defined in `database.h`, which defines what kind of information is passed
between them.

# How to compile and run

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
