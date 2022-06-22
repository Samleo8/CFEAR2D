#!/bin/bash

OUTPUT_NAME=${1%/}

if [ -z $OUTPUT_NAME ]; then
    echo "USAGE: ./mp4-from-folder.sh <folder-name> [dataset-id:0] [start_number:0] [frame-rate]"
    exit
fi

DATASET_ID=${2:-0}
START_NUM=${3:-0}
FRAME_RATE=${4:-10}
DO_SCALE=${5:-1} # scale by default

OUTPUT_BASE_FOLDER=$OUTPUT_NAME/$DATASET_ID

if [[ $DO_SCALE == "1" ]]; then
    SCALE=-vf "scale=trunc(iw/4)*2:trunc(ih/4)*2"
else
    SCALE=""
fi

ffmpeg -y -start_number $START_NUM -framerate $FRAME_RATE -i $OUTPUT_BASE_FOLDER/%d.jpg $SCALE -c:v libx265 -loop -1 -crf 32 $OUTPUT_BASE_FOLDER/$OUTPUT_NAME.mp4