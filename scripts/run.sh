#!/bin/bash

DATASET_ID=${1:-0}
echo "[Dataset $DATASET_ID]"

# IMG_IND=0
# ./build/TestRadar 0 $IMG_IND 1
# exit 0

START_IND=${2:-0}
END_IND=${3:-10}

for (( IMG_IND=$START_IND; IMG_IND<=$END_IND; IMG_IND++ )); do
    echo " > Running on image $IMG_IND"
    ./build/TestRadar $DATASET_ID $IMG_IND 1
done

./scripts/mp4-from-folder.sh results $DATASET_ID $START_IND 5