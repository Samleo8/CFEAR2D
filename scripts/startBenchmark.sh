#!/bin/bash

DATASET_ID=${1:-0}
START_IND=${2:-0}
END_IND=${3:--1}

if [[ $START_IND != 0 || $END_IND != -1 ]]; then
    echo "For now, benchmark script can only handle FULL sequences (i.e. 0 to -1)."
    exit 1
fi

BENCHMARK_DIR=./rpg_trajectory_evaluation
RESULTS_DIR=./results/$DATASET_ID/poses

if [ ! -f ${RESULTS_DIR}/stamped_groundtruth.txt ]; then
    echo "Ground Truth file not found!"
    exit 1
elif [ ! -f ${RESULTS_DIR}/stamped_traj_estimate.txt ]; then
    echo "Estimated poses not found!"
    exit 1
fi

# Generate YAML file
generateYAMLFile() {
    YAML_FILE=${RESULTS_DIR}/eval_cfg.yaml
    
    echo "align_type: se3" > $YAML_FILE
    echo "align_num_frames: -1" >> $YAML_FILE
}

generateYAMLFile

echo "Benchmarking on dataset ${DATASET_ID}..."
python ${BENCHMARK_DIR}/scripts/analyze_trajectory_single.py --png $RESULTS_DIR
echo "Done!"
echo ""
echo "Results can be found in the ${RESULTS_DIR}/saved_results and .../plots folders"