import numpy as np
import sys, os
from plotORSP import plotORSPPoint, plt


def parseORSPData(dataset: str, frame: int) -> (np.ndarray, np.ndarray):
    '''
    @brief Extracts center and normal information out of file containing ORSP points
    '''
    filePath = os.path.join('results', str(dataset), 'orsp',
                            'orsp_' + str(frame) + '.txt')
    with open(filePath, 'r') as f:
        lines = f.readlines()

        data = []
        for i in range(len(lines)):
            line = lines[i].strip().replace("Center: ", "").replace(" | Normal:", "")
            data.append(line.split(' '))

        data_np = np.array(data, dtype=np.float64)

        dim = data_np.shape[1] // 2
        centers = data_np[:, :dim]
        normals = data_np[:, dim:]

    return (centers, normals)


if __name__ == "__main__":
    dataset = "0" if len(sys.argv) <= 1 else sys.argv[1]
    startInd = 0 if len(sys.argv) <= 2 else int(sys.argv[2])
    endInd = 10 if len(sys.argv) <= 3 else int(sys.argv[3])

    plt.clf()

    prev_centers, prev_normals = parseORSPData(dataset, startInd)

    for i in range(startInd + 1, endInd):
        plt.title(f"Frame {i}")
        centers, normals = parseORSPData(dataset, i)

        plotORSPPoint(prev_centers, prev_normals, show_normals=True, alpha=0.1)
        plotORSPPoint(centers, normals, show_normals=True, alpha=0.8)

        prev_centers = centers
        prev_normals = normals

        plt.waitforbuttonpress()