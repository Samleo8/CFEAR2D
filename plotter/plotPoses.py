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


def plotPosesVideo(poses: np.ndarray, startFrame=0, pauseInterval=0.05):
    N = poses.shape[0]
    for i in range(1, N):
        plt.clf()
        plt.title(f'Frame {startFrame + i}')
        plotPoses(poses[:i + 1, :], show=False)
        plt.pause(pauseInterval)

    return