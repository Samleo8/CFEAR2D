#!/bin/bash

cd build
cmake --build . || exit 1
cd ..