#!/bin/bash

# IMG_IND=30
# ./build/TestRadar 0 $IMG_IND 1
# exit 0

START_IND=0
END_IND=30
for (( IMG_IND=$START_IND; IMG_IND<=$END_IND; IMG_IND++ )); do
    echo $IMG_IND
    ./build/TestRadar 0 $IMG_IND 1
done