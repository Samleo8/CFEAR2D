#!/bin/bash

DATASET_ID=${1:-0}
TARGET=${2:-"main"}
START_IND=${3:-0}
END_IND=${4:--1}

if [[ ! -d ./data/$DATASET_ID ]]; then
    echo "Dataset $DATASET_ID not found! Aborting!"
    exit 1
fi

echo "[Dataset $DATASET_ID]"

if [[ $TARGET == "main" || $TARGET == "cfear" || $TARGET == "feed" ]]; then
    if [[ $START_IND == 0 && $END_IND == -1 ]]; then
        echo "NOTE: Running on full sequence! Benchmarks will be performed after completion."
    fi

    ./build/RunCFEAR $DATASET_ID $START_IND $END_IND || exit 1
    ./scripts/run.sh $DATASET_ID plotter $START_IND $END_IND

    if [[ $START_IND == 0 && $END_IND == -1 ]]; then
        ./scripts/run.sh $DATASET_ID benchmark $START_IND $END_IND
    fi
elif [[ $TARGET == "radar" || $TARGET == "video" || $TARGET == "filter" ]]; then
    for ((IMG_IND = $START_IND; IMG_IND <= $END_IND; IMG_IND++)); do
        echo " > Running on image $IMG_IND"
        ./build/TestRadar $DATASET_ID $IMG_IND 1 || exit 1
    done

    FPS=10
    ./scripts/mp4-from-folder.sh results $DATASET_ID $START_IND $END_IND $FPS
elif [[ $TARGET == "plotter" || $TARGET == "plot" ]]; then
    python plotter/parsePoses.py $DATASET_ID $START_IND $END_IND 1
elif [[ $TARGET == "plotter_nogt" || $TARGET == "plot_nogt" ]]; then
    python plotter/parsePoses.py $DATASET_ID $START_IND $END_IND 0
elif [[ $TARGET == "benchmark" ]]; then
    ./scripts/run.sh $DATASET_ID gt || exit 1
    ./scripts/run.sh $DATASET_ID kitti -1 || exit 1
    ./scripts/run.sh $DATASET_ID kitti $START_IND $END_IND || exit 1

    rm -rf results/poses/plots
    rm -rf results/poses/saved_results
    
    ./scripts/startBenchmark.sh $DATASET_ID $START_IND $END_IND || exit 1
elif [[ $TARGET == "kitti" ]]; then
    ./build/PosesToKITTI $DATASET_ID $START_IND $END_IND
elif [[ $TARGET == "groundtruth" || $TARGET == "gt" || $TARGET == "procgt" ]]; then
    ./build/ProcessGroundTruth $DATASET_ID || exit 1
elif [[ $TARGET == "debug" ]]; then
    ./build/TestCostFunction $DATASET_ID $START_IND || exit 1
    python plotter/parseORSP.py $DATASET_ID $START_IND $(($START_IND + 3))
elif [[ $TARGET == "keyframe" ]]; then
    ./build/TestKeyframe $DATASET_ID $START_IND $END_IND || exit 1
    ./scripts/run.sh $DATASET_ID plotter_nogt $START_IND $END_IND
elif [[ $TARGET == "both" ]]; then
    ./scripts/run.sh $DATASET_ID main $START_IND $END_IND
    ./scripts/run.sh $DATASET_ID radar $START_IND $END_IND
else
    echo "Unknown target $TARGET"
    exit 1
fi
