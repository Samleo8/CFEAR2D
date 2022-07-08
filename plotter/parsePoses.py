import numpy as np
import os, sys
from plotPoses import plotPoses, plt

def parsePoses(filePath: str) -> np.ndarray:
    poseInfo = []
    poseInfo.append([0, 0, 0, 1])  # x, y, theta, kf

    with open(filePath, 'r') as f:
        data = f.readline()

        kf = data.startswith('kf')
        data = data.replace('kf ', '').replace('Pose2D: ', '')
        dataArr = list(map(float, data.split(' ')))
        dataArr.append(kf)

        poseInfo.append(dataArr)

    return poseInfo


if __name__ == '__main__':
    dataset = "0" if len(sys.argv) <= 1 else sys.argv[1]
    startInd = 0 if len(sys.argv) <= 2 else int(sys.argv[2])
    endInd = 10 if len(sys.argv) <= 3 else int(sys.argv[3])

    filePath = os.path.join('results', dataset,
                            f'poses_{startInd}_{endInd}.txt')

    poseArr = parsePoses(filePath)
    print(poseArr)

    plotPoses(poseArr)

    plt.tight_layout()
    plt.show()