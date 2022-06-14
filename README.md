# PhaRaO

C++ Implementation of [PhaRaO paper](https://irap.kaist.ac.kr/publications/yspark-2020-icra.pdf)

- [PhaRaO](#pharao)
  - [Requirements](#requirements)
  - [Build](#build)
    - [Windows](#windows)
    - [Linux](#linux)
  - [Documentation](#documentation)
  - [Data Reformatting](#data-reformatting)
  - [Radar Image Format](#radar-image-format)
  - [Visualiser](#visualiser)
    - [Usage](#usage)
  - [Radar Keyframe Odometry Tester](#radar-keyframe-odometry-tester)
    - [Usage](#usage-1)
  - [Other Resources](#other-resources)
    - [Robot Car SDK (Viewing Toolkit)](#robot-car-sdk-viewing-toolkit)
    - [FMT](#fmt)
      - [Papers](#papers)
      - [Implementation](#implementation)

## Requirements

OpenCV >= 3.4.11

## Build

To begin, you should first create a `build` directory.

### Windows

`cmd > cd build; cmake -D OPENCV_DIR=<path_to_opencv_dir> ..`

**Example: `cd build; cmake -D OPENCV_DIR=C:\Users\Intern\Downloads\OpenCV\Build\opencv .`**

Alternatively, set your OPENCV_DIR environment variable by using

`setx OPENCV_DIR <path_to_opencv_dir>`

_NOTE:_ It may be wise to open CMake-GUI to see the exact variables, in case your system overrides some variables.

### Linux

```bash
$ cmake . && make
```

## Documentation

Doxygen docs can be found at `./docs/doxygen/html/index.html`. You can regenerate them using the `./doxygen` command.

## Data Reformatting

Data is being reformatted as:

```bash
data
 |
 |---0
     |
     |---radar
           |---<img_1>.jpg
          ...
           |---<img_n>.jpg
   | radar.timestamps
 |
 |---1
    ...
```

## Radar Image Format

Each image is 3779 x 40 (range x angle). Angle resolution is 0.9 degrees. Range 0 to 163m, resolution of 4.38cm. More info [here](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/documentation).

## Visualiser

The included `TestVisualiser` module gives full visualisation of the radar image, ground truth and prediction odometry. It can be run from the `./TestVisualiser` or `./build/Release/TestVisualiser.exe` executable. Follow the instructions on the screen (that show for 3 seconds), that tell you the keyboard shortcuts for pausing, stepping through, etc.

### Usage

```bash
./TestVisualiser [dataset [startFrame [endFrame [0/1:outputToFile]]]]
```

- `dataset`: Which particular dataset to visualise
- `startFrame`: Frame to start at
- `endFrame`: Frame to end at (-1 to just go all the way to the end)
- `outputToFile`: Flag of whether or not output should be piped to a file in the `./raw_output` for future processing. This is used for debugging. If python is available, the `TestVisualiser` executable will automatically run the debugger files `./raw_output/txt_to_json.py` and `./raw_output/error_check.py` to generate a CSV file for debugging purposes.

## Radar Keyframe Odometry Tester

The included `TestRadar` module also allows for the user to check the results of the algorithm for arbitrary keyframe to keyframe odometry. One can think of this module as a subset of the [Visualiser](#visualiser) module, except that we are only outputting the results of odometry between 2 specified keyframe images. Also, for debugging and visualisation purposes, the rotated image and numerical is shown side by side with the 2 images in a single image.

### Usage

```bash
./TestRadar <dataset> <image1ID> <image2ID> [filter size [0|1:saveDirectlyToFile]]
```

- `dataset`: Which particular dataset to visualise
- `image1ID`, `image2ID`: Image IDs to compare against. Odometry will be calculated from `image1ID` against `image2ID`, rotating `image1ID` to fit the frame of `image2ID` as done in the actual algorithm.
- `filterSize`: Size of high-pass filter to use. Default: 150
- `saveDirectlyToFile`: Flag to specify that instead of displaying the radar images on the screen, directly save them to `./raw_output/error_images` folder. If set, the program also generates a series of concatenated predictions for frame to frame odometry in the range `[image1ID, image2ID]` in a single output image.

## Other Resources

### Robot Car SDK (Viewing Toolkit)

I modified the [original robot car SDK toolkit](https://github.com/ori-mrg/robotcar-dataset-sdk) to support pausing, playing, stepping through etc. ~~It is included as a submodule repo under `robotcar-dataset-sdk`.~~

To play the radar feed, clone my git repo then run the internal script.

```bash
$ git clone git@github.com:Samleo8/robotcar-dataset-sdk.git
$ ./robotcar-dataset-sdk/playRadar
```

Note that you will require Python >= 3.4 to run it.

### FMT

#### Papers

![FMT Algorithm](./theory/fmt_algo.jpg)

[1] P. Checchin, F. Gérossier, C. Blanc, R. Chapuis, and L. Trassoudaine, “Radar scan matching slam using the fourier-mellin transform,” in Field and Service Robotics. Springer, 2010, pp. 151–161. [(link)](./theory/Radar_Scan_Matching_SLAM_Using_the_Fourier-Mellin.pdf)

#### Implementation

[MatLab Implementation](https://www.mathworks.com/matlabcentral/fileexchange/19731-fourier-mellin-image-registration)

[Python Implementation of FMT/Phase Co-relation](https://github.com/polakluk/fourier-mellin)

[Possible C++ Implementation](https://github.com/Smorodov/LogPolarFFTTemplateMatcher)
