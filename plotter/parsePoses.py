import numpy as np
import os, sys
from plotPoses import plotPoses, plotPosesVideo, plt


def parsePoses(filePath: str) -> np.ndarray:
    poseInfo = []
    poseInfo.append([0, 0, 0, 1])  # x, y, theta, kf

    with open(filePath, 'r') as f:
        for data in f.readlines():
            kf = int(data.strip().endswith('kf'))
            
            data = data.replace('Pose2D: ', '').replace(' kf', '')
            dataArr = list(map(float, data.split(' ')))
            dataArr.append(kf)

            poseInfo.append(dataArr)

    return np.array(poseInfo).astype(np.float32)


def getImagePaths(dataset, startInd, endInd):
    imgBasePath = os.path.join('results', str(dataset))

    imagePaths = []
    for i in range(startInd, endInd):
        imgPath = os.path.join(imgBasePath, f'{i}.jpg')
        imagePaths.append(imgPath)

    return imagePaths


if __name__ == '__main__':
    dataset = "0" if len(sys.argv) <= 1 else sys.argv[1]
    startInd = 0 if len(sys.argv) <= 2 else int(sys.argv[2])
    endInd = 10 if len(sys.argv) <= 3 else int(sys.argv[3])
    videoMode = False if len(sys.argv) <= 4 else bool(int(sys.argv[4]))
    startFrame = startInd if len(sys.argv) <= 5 else int(sys.argv[5])

    baseResultsPath = os.path.join('results', dataset, 'poses')
    filePath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')

    poseArr = parsePoses(filePath)
    print(poseArr)

    # print(poseArr)
    if videoMode:
        imagePaths = getImagePaths(dataset, startInd, endInd)
        plotPosesVideo(poseArr,
                       imagePaths,
                       startInd=startInd,
                       startFrame=startFrame,
                       pauseInterval=-1)
    else:
        plotPoses(poseArr)

        plt.tight_layout()

        plt.savefig(filePath.replace('.txt', '.jpg'))
        # plt.show()