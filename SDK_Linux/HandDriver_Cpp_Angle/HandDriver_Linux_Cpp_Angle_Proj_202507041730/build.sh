#!/bin/bash 

mkdir build
cd build
cmake ..
make
cd ..

sudo cp lib/libjsoncpp.so /usr/lib
sudo cp lib/libjsoncpp.so.27 /usr/lib
sudo cp build/libUDEServer.so /usr/lib

g++ main.cxx -o ServerSample -lpthread -lUDEServer -ljsoncpp
