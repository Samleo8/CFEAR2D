import numpy as np
import sys, os
import matplotlib.pyplot as plt
from plotter.parsePoses import parsePoses

def convertPoses(dataset, startInd, endInd):
    baseResultsPath = os.path.join('results', dataset, 'poses')
    baseOutputResultsPath = os.path.join('results', dataset, 'poses_kitti')

    filePath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')
    outputPath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')

    poseArr = parsePoses(filePath)

if __name__ == '__main__':
    dataset = "0" if len(sys.argv) <= 1 else sys.argv[1]
    startInd = 0 if len(sys.argv) <= 2 else int(sys.argv[2])
    endInd = 10 if len(sys.argv) <= 3 else int(sys.argv[3])

    converted_poses = convertPoses(dataset, startInd, endInd)
