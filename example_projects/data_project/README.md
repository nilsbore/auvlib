# Example project

## Compiling

First, create a directory named `gpgs_build` within this directory:
```
mkdir gpgs_build
cd gpgs_build
```
Then, run cmake using:
```
cmake -DCMAKE_INSTALL_PREFIX=../gpgs_install ../../..
```
followed by make: `make && make install`.
Now, go to the `data_project` directory and create a build folder for that project:
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
