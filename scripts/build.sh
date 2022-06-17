#!/bin/bash

rm -rf build
mkdir build
cd build

if [[ -z $OpenCV_DIR ]]; then
    cmake .. && cmake --build .
else
    cmake -D OpenCV_DIR=$OpenCV_DIR .. && cmake --build .
fi

cd ..