#!/bin/bash

cd build
cmake --build .
./TestRadar 0 2300
cd ..