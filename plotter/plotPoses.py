from socket import TIPC_SUBSCR_TIMEOUT
import numpy as np
import matplotlib.pyplot as plt


def plotPoses(poses: np.ndarray,
              gt: bool = False,
              forceSquare: bool = True,
              show: bool = False):
    '''
    @brief Plot poses
    @param[in] poses Poses array [x | y | theta | kf]
    @param[in] gt Whether poses represent ground truth (affects labelling etc)
    @param[in] forceSquare Whether to force square aspect ratio
    @param[in] show Whether to show the plot
    '''
    x = poses[:, 0]
    y = poses[:, 1]
    theta = poses[:, 2]
    kf = poses[:, 3].astype(bool)
    kfMask = (kf == True)

    label = 'GT' if gt else 'Pred'
    color = 'green' if gt else 'blue'

    plt.plot(x, y, marker='o', color=color, alpha=0.4, label=label)

    if not gt:
        plt.scatter(x[kfMask], y[kfMask], color='red')

    plt.xlabel('x [m]')
    plt.ylabel('y [m]')

    # Force a square
    if forceSquare:
        xCurr = plt.xlim()
        yCurr = plt.ylim()

        mn = np.floor(min(x.min(), y.min(), xCurr[0], yCurr[0]))
        mx = np.ceil(max(x.max(), y.max(), xCurr[1], yCurr[1]))
        plt.xlim(mn, mx)
        plt.ylim(mn, mx)

    if show:
        plt.show()

    return


def percentCenterCrop(img: np.ndarray, percent=0.7) -> np.ndarray:
    '''
    @brief Zoom crop a percent of an image, from center
    @param[in] img Image to crop
    @param[in] percent Percent to crop. Larger = more zoom
    
    @return Cropped image
    '''
    h, w = img.shape[:2]
    h_crop = int(h * percent / 2)
    w_crop = int(w * percent / 2)

    return img[h_crop:-h_crop, w_crop:-w_crop, :]

video_params = {
    'pause': False,
    'quit': False,
    'pauseInterval': 0.05
}

def keypress_handler(event):
    if event.key == 'q':
        video_params['quit'] = True
    elif event.key == '0':
        video_params['pauseInterval'] = -1
    elif event.key == '1':
        video_params['pauseInterval'] = 0.01
    elif event.key == 'p' or event.key == ' ':
        if video_params['pauseInterval'] != -1:
            video_params['pause'] = not video_params['pause']
            print("Video paused:", video_params['pause'])

def plotPosesVideo(poses: np.ndarray,
                   imagePaths: np.ndarray = None,
                   startInd: int = 0,
                   startFrame: int = 0,
                   pauseInterval: float = 0.01):
    N = poses.shape[0]

    rows = 1
    cols = 1 if imagePaths is None else 2

    video_params['pauseInterval'] = pauseInterval

    for i in range(startFrame - startInd + 1, N):
        plt.clf()

        plt.ion()
        plt.gcf().canvas.mpl_connect('key_press_event', keypress_handler)

        plt.suptitle(f'Frame {startInd + i}')

        plt.subplot(rows, cols, 1)
        plt.title(f'Pose: {poses[i, :-1]}')
        plotPoses(poses[:i + 1, :], show=False)

        if imagePaths is not None:
            plt.subplot(rows, cols, 2)
            plt.title('Radar Image (Zoomed)')
            img = plt.imread(imagePaths[i])
            plt.imshow(percentCenterCrop(img), aspect='auto')
            plt.axis('off')

        plt.tight_layout()

        # plt.show()
        plt.waitforbuttonpress(video_params['pauseInterval'])

        if video_params['quit']:
            break
        else:
            if video_params['pause']:
                plt.waitforbuttonpress(-1)

    return