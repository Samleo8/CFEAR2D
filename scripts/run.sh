#!/bin/bash

DATASET_ID=${1:-0}
TARGET=${2:-"radar"}
START_IND=${3:-0}
END_IND=${4:-10}

if [[ ! -d ./data/$DATASET_ID ]]; then
    echo "Dataset $DATASET_ID not found! Aborting!"
    exit 1
fi

echo "[Dataset $DATASET_ID]"

if [[ $TARGET == "keyframe" ]]; then
    ./build/TestKeyframe 0 $START_IND $END_IND
elif [[ $TARGET == "radar" ]]; then
    for (( IMG_IND=$START_IND; IMG_IND<=$END_IND; IMG_IND++ )); do
        echo " > Running on image $IMG_IND"
        ./build/TestRadar $DATASET_ID $IMG_IND 1 || exit 1
    done

    FPS=10
    ./scripts/mp4-from-folder.sh results $DATASET_ID $START_IND $FPS
else
    echo "Unknown target $TARGET"
    exit 1
fi