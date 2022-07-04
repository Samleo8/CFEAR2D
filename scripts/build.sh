#!/bin/bash

rm -rf build
mkdir build
cd build

if [[ -z $OpenCV_DIR ]]; then
    cmake -D CMAKE_EXPORT_COMPILE_COMMANDS=1 .. && cmake --build .
else
    cmake -D CMAKE_EXPORT_COMPILE_COMMANDS=1 -D OpenCV_DIR=$OpenCV_DIR .. && cmake --build .
fi

cd ..