#!/bin/bash

OUTPUT_NAME=${1%/}

if [ -z $OUTPUT_NAME ]; then
    echo "USAGE: ./mp4-from-folder.sh <folder-name> [dataset-id:0] [start_number:0] [frame-rate]"
    exit
fi

DATASET_ID=${2:-0}
START_NUM=${3:-0}
FRAME_RATE=${4:-60}

OUTPUT_BASE_FOLDER=$OUTPUT_NAME/$DATASET_ID

ffmpeg -y -start_number $START_NUM -framerate $FRAME_RATE -i $OUTPUT_BASE_FOLDER/%d.jpg -loop -1 -profile:v high -crf 28 -pix_fmt yuv420p $OUTPUT_BASE_FOLDER/$OUTPUT_NAME.mp4