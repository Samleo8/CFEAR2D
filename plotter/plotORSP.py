import numpy as np
import sys, os
import matplotlib.pyplot as plt

def plotORSPPoint(centers: np.array,
                  normals: np.ndarray,
                  show_normals: bool = True,
                  center_color: str = "red",
                  normal_color: str = "green",
                  alpha=1):
    # First draw the ORSP point centers
    plt.scatter(centers[:, 0],
                centers[:, 1],
                c=center_color,
                marker='.',
                alpha=alpha)

    if show_normals:
        # Then draw the arrows
        for i in range(normals.shape[0]):
            plt.arrow(centers[i, 0],
                      centers[i, 1],
                      normals[i, 0],
                      normals[i, 1],
                      color=normal_color,
                      alpha=alpha)

    return
