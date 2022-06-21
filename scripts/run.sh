#!/bin/bash

# IMG_IND=30
# ./build/TestRadar 0 $IMG_IND 1

END_IND=10
for (( IMG_IND=$START; IMG_IND<=$END; IMG_IND++ )); do
    echo $IMG_IND
    ./build/TestRadar 0 $IMG_IND 1
done