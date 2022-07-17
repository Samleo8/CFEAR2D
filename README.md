# CFEAR

C++ Implementation of [CFEAR paper](https://arxiv.org/pdf/2105.01457.pdf)

- [CFEAR](#cfear)
  - [Requirements](#requirements)
  - [Build](#build)
    - [Linux](#linux)
    - [Windows](#windows)
  - [Data Reformatting](#data-reformatting)
    - [Radar Image Format](#radar-image-format)
  - [Universal Run Script](#universal-run-script)
    - [Target Names](#target-names)
  - [Code Documentation](#code-documentation)
  - [Other Resources](#other-resources)
    - [Robot Car SDK (Viewing Toolkit)](#robot-car-sdk-viewing-toolkit)

## Requirements

OpenCV >= 3.4.11
Eigen3 >= 3.3.7
Ceres >= 2.2

## Build

### Linux

To build, simply run the `./scripts/build.sh` bash script.

If needed, set the `OpenCV_DIR` variable that points to your OpenCV directory in your `.bashrc` or `.bash_profile` file, and re-run the build script.

### Windows

If possible, you might be able to run the bash script via Powershell. Otherwise, manually create a `./build` folder, `cd` into it, and run `cmake ..` via the command line or GUI. Then, to compile all the related subprograms, run `cmake --build .`.

## Data Reformatting

Data is being reformatted as:

```bash
data
 |
 |---0
    |---gt
          |---radar_odometry.csv
    |---radar
          |---<img_1>.jpg
              
               ...
               
          |---<img_n>.jpg
    | radar.timestamps
 |
 |---1
    ...
```

### Radar Image Format

Each image is 3779 x 40 (range x angle). Angle resolution is 0.9 degrees. Range 0 to 163m, resolution of 4.38cm.
The raw radar image also contains 11 columns of metadata such as timestamps and azimuths, handled by the algorithm.

More info [here](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/documentation).

## Universal Run Script

There are many subprograms that are built and are available for use. To run them, there is a universal run script that can be used as follows:

```bash
./scripts/run.sh [DATASET_ID=0 [TARGET="main" [START_IND=0 [END_IND=-1]]]]
```

- `DATASET_ID`: ID of dataset as integer, as formatted in [#data-formatting](Data Formatting) section above.
- `TARGET`: Type of sub-program to run, as string (full list below)
- `START_IND`: Index of starting frame (might not be used, depends on `TARGET`)
- `END_IND`: Index of ending frame (might not be used, depends on `TARGET`). `-1` implies run to completion.

### Target Names

Below is the full list of possible subprograms to run.

| `TARGET` String     | Description                                                 |
|--------------------:|:-----------:                                                |
| `cfear`, `main`	    | Main program. Generates visualization of predicted and ground truth poses in `./results/<DATASET_ID>/poses/poses_<START_IND>_<END_IND>.jpg`. Raw pose information found in `./results/<DATASET_ID>/poses/poses_<START_IND>_<END_IND>.txt` |
| `radar`, `video`, `filter` | Visualization of filtering and oriented surface points, with video generation. Images generated in `./results/<DATASET_ID>` folder as `<FRAME_NUM>.jpg`. MP4 video generated as `results_<START_IND>_<END_IND>.mp4` (requires FFMPEG). |

## Code Documentation

Doxygen docs can be found at `./docs/doxygen/html/index.html`. You can regenerate them using the `./doxygen` command.


## Other Resources

### Robot Car SDK (Viewing Toolkit)

I modified the [original robot car SDK toolkit](https://github.com/ori-mrg/robotcar-dataset-sdk) to support pausing, playing, stepping through etc. ~~It is included as a submodule repo under `robotcar-dataset-sdk`.~~

To play the radar feed, clone my git repo then run the internal script.

```bash
git clone git@github.com:Samleo8/robotcar-dataset-sdk.git
./robotcar-dataset-sdk/playRadar
```

Note that you will require Python >= 3.4 to run it.