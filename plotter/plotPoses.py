from tracemalloc import start
import numpy as np
import matplotlib.pyplot as plt


def plotPoses(poses: np.ndarray, show: bool = False):
    '''
    @brief Plot poses
    @param[in] poses Poses array [x | y | theta | kf]
    '''
    x = poses[:, 0]
    y = poses[:, 1]
    theta = poses[:, 2]
    kf = poses[:, 3].astype(bool)
    kfMask = (kf == True)

    plt.plot(x, y, marker='o', color='blue', alpha=0.4)
    plt.scatter(x[kfMask], y[kfMask], color='red')

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    # Force a square
    mn = np.floor(min(x.min(), y.min()))
    mx = np.ceil(max(x.max(), y.max()))
    plt.xlim(mn, mx)
    plt.ylim(mn, mx)

    if show:
        plt.show()

    return


def plotPosesVideo(poses: np.ndarray,
                   imagePaths: np.ndarray = None,
                   startInd: int = 0,
                   startFrame: int = 0,
                   pauseInterval: float = 0.05):
    N = poses.shape[0]

    rows = 1
    cols = 1 if imagePaths is None else 2

    for i in range(startFrame - startInd + 1, N):
        plt.clf()
        plt.suptitle(f'Frame {startInd + i}')

        plt.subplot(rows, cols, 1)
        plt.title(f'Pose: {poses[i, :-1]}')
        plotPoses(poses[:i + 1, :], show=False)

        if imagePaths is not None:
            plt.subplot(rows, cols, 2)
            plt.title('Radar Image')
            plt.imshow(plt.imread(imagePaths[i]))
            plt.axis('off')

        plt.tight_layout()

        if pauseInterval <= 0:
            plt.waitforbuttonpress()
        else:
            plt.pause(pauseInterval)

    return