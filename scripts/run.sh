#!/bin/bash

END_IND=5
for IMG_IND in {0..$END_IND}; do
    ./build/TestRadar 0 $IMG_IND 1
done