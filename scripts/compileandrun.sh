#!/bin/bash

cd build
cmake --build .
cd ..

./scripts/run.sh