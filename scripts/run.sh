#!/bin/bash

DATASET_ID=0
echo "[Dataset $DATASET_ID]"

IMG_IND=0
./build/TestRadar 0 $IMG_IND 0
exit 0

# START_IND=0
# END_IND=10
# for (( IMG_IND=$START_IND; IMG_IND<=$END_IND; IMG_IND++ )); do
#     echo " > Running on image $IMG_IND"
#     ./build/TestRadar $DATASET_ID $IMG_IND 1
# done