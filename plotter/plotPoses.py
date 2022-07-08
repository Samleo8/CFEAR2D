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
    kf = poses[:, 3]
    kfMask = bool(kf)

    plt.plot(x, y, marker='o', color='blue', alpha=0.4)
    plt.scatter(x[kfMask], y[kfMask], color='red')

    if show:
        plt.show()

    return