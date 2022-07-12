import numpy as np
import os, sys
from plotPoses import plotPoses, plotPosesVideo, plt


def parsePoses(filePath: str) -> np.ndarray:
    poseInfo = []
    poseInfo.append([0, 0, 0, 1])  # x, y, theta, kf

    with open(filePath, 'r') as f:
        for data in f.readlines():
            kf = data.startswith('kf')
            data = data.replace('kf ', '').replace('Pose2D: ', '')
            dataArr = list(map(float, data.split(' ')))
            dataArr.append(kf)

            poseInfo.append(dataArr)

    return np.array(poseInfo).astype(np.float32)


if __name__ == '__main__':
    dataset = "0" if len(sys.argv) <= 1 else sys.argv[1]
    startInd = 0 if len(sys.argv) <= 2 else int(sys.argv[2])
    endInd = 10 if len(sys.argv) <= 3 else int(sys.argv[3])
    videoMode = False if len(sys.argv) <= 4 else bool(int(sys.argv[4]))

    baseResultsPath = os.path.join('results', dataset)
    filePath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')

    poseArr = parsePoses(filePath)
    # print(poseArr)
    if videoMode:
        plotPosesVideo(poseArr)
    else:
        plotPoses(poseArr)

        plt.tight_layout()

        plt.savefig(filePath.replace('.txt', '.jpg'))
        plt.show()