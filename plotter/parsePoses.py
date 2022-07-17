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
    forceSquare = True if len(sys.argv) <= 4 else int(sys.argv[4])
    videoMode = False if len(sys.argv) <= 5 else bool(int(sys.argv[5]))
    startFrame = startInd if len(sys.argv) <= 6 else int(sys.argv[6])

    print(
        "USAGE: python3 parsePoses.py [dataset=0 [startInd=0 [endInd=10 [forceSquare=1 [videoMode=0 [startFrame=startInd]]]]]]"
    )

    baseResultsPath = os.path.join('results', dataset, 'poses')

    # Get normal poses
    filePath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')
    poseArr = parsePoses(filePath)

    # Get GT poses
    filePathGT = os.path.join(baseResultsPath, 'gt.txt')
    gtPoseArr = parsePoses(filePathGT)

    if videoMode:
        imagePaths = getImagePaths(dataset, startInd, endInd)
        plotPosesVideo(poseArr,
                       imagePaths,
                       startInd=startInd,
                       startFrame=startFrame,
                       pauseInterval=-1)
    else:
        plotPoses(poseArr, forceSquare=forceSquare)

        if endInd == -1:
            gtPoseArrTrunc = gtPoseArr[startInd:]
        else:
            gtPoseArrTrunc = gtPoseArr[startInd:endInd]

        plotPoses(gtPoseArrTrunc, gt=True, forceSquare=forceSquare)

        plt.tight_layout()

        plt.savefig(filePath.replace('.txt', '.jpg'))
        # plt.show()