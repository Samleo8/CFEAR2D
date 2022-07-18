import numpy as np
import os, sys
from plotPoses import plotPoses, plotPosesVideo, plt


def getRotMat(angle: float) -> np.ndarray:
    rotMat = np.array([[np.cos(angle), -np.sin(angle)],
                       [np.sin(angle), np.cos(angle)]])
    return rotMat


def parsePoses(filePath: str) -> np.ndarray:
    poseInfo = []
    poseInfo.append([0, 0, 0, 1])  # x, y, theta, kf

    with open(filePath, 'r') as f:
        for data in f.readlines():
            kf = int(data.strip().endswith('kf') or data.startswith('kf'))

            data = data.replace('Pose2D: ', '').replace('kf', '').strip()
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
    plotGT = True if len(sys.argv) <= 4 else bool(int(sys.argv[4]))
    forceSquare = True if len(sys.argv) <= 5 else bool(int(sys.argv[5]))
    videoMode = False if len(sys.argv) <= 6 else int(sys.argv[6])
    startFrame = startInd if len(sys.argv) <= 7 else int(sys.argv[7])

    print("====================PARSING POSES====================")
    print(
        "USAGE: python3 parsePoses.py [dataset=0 [startInd=0 [endInd=10 [plotGT=1 [forceSquare=1 [videoMode=0 [startFrame=startInd]]]]]]]"
    )

    baseResultsPath = os.path.join('results', dataset, 'poses')

    # Get normal poses
    filePath = os.path.join(baseResultsPath, f'poses_{startInd}_{endInd}.txt')
    poseArr = parsePoses(filePath)

    # Get GT poses
    if plotGT:
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

        if plotGT:
            if endInd == -1:
                gtPoseArrTrunc = gtPoseArr[startInd:]
            else:
                gtPoseArrTrunc = gtPoseArr[startInd:endInd]

            # Need to force starting world pose as (0,0,0)
            # While accounting for proper orientation
            angle = gtPoseArrTrunc[0][2]
            rotMatInv = getRotMat(angle).T

            # Translate
            gtPoseArrTrunc -= gtPoseArrTrunc[0]

            # Rotate
            gtPoseArrTrunc[:, :2] = (rotMatInv @ gtPoseArrTrunc[:, :2].T).T

            # Plot the correct ground truth poses
            plotPoses(gtPoseArrTrunc, gt=True, forceSquare=forceSquare)
            plt.tight_layout()

        imgSavePath = filePath.replace('.txt', '.jpg')
        plt.savefig(imgSavePath)

        print("\nFigure saved to", imgSavePath)
        # plt.show()