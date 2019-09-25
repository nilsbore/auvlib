# Example project

## Compiling

First, create a directory named `build` within the auvlib project:
```
mkdir build
cd build
```
Then, run cmake using:
```
cmake -DCMAKE_INSTALL_PREFIX=../install ..
```
followed by make: `make && make install`.
Now, go to the `complete_project` directory and create a build folder for that project:
```
cd ..
mkdir build
cd build
```
Now, run cmake using
```
PATH=$PATH:../../../install/share/ cmake ..
```
followed by make: `make`.
Now everything should have compiled.
