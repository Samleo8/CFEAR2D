#!/bin/bash

DATASET_ID=${1:-0}
TARGET=${2:-"main"}
START_IND=${3:-0}
END_IND=${4:-10}

if [[ ! -d ./data/$DATASET_ID ]]; then
    echo "Dataset $DATASET_ID not found! Aborting!"
    exit 1
fi

echo "[Dataset $DATASET_ID]"

if [[ $TARGET == "both" ]]; then
    ./scripts/run.sh $DATASET_ID keyframe $START_IND $END_IND
    ./scripts/run.sh $DATASET_ID radar $START_IND $END_IND
elif [[ $TARGET == "main" || $TARGET == "cfear" ]]; then
    ./build/RunCFEAR $DATASET_ID $START_IND $END_IND
    python plotter/parsePoses.py $DATASET_ID $START_IND $END_IND
elif [[ $TARGET == "debug" ]]; then
    ./build/TestCostFunction $DATASET_ID $START_IND
    python plotter/parseORSP.py $DATASET_ID $START_IND $(( $START_IND + 3 ))
elif [[ $TARGET == "keyframe" ]]; then
    ./build/TestKeyframe $DATASET_ID $START_IND $END_IND
    python plotter/parsePoses.py $DATASET_ID $START_IND $END_IND
elif [[ $TARGET == "radar" ]]; then
    for ((IMG_IND = $START_IND; IMG_IND <= $END_IND; IMG_IND++)); do
        echo " > Running on image $IMG_IND"
        ./build/TestRadar $DATASET_ID $IMG_IND 1 || exit 1
    done

    FPS=10
    ./scripts/mp4-from-folder.sh results $DATASET_ID $START_IND $END_IND $FPS
else
    echo "Unknown target $TARGET"
    exit 1
fi
