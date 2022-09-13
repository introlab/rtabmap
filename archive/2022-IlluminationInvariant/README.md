
## Multi-Session Visual SLAM for Illumination Invariant Re-Localization in Indoor Environments

* Paper: https://doi.org/10.3389/frobt.2022.801886

* The setup: we did 6 mapping sessions at dusk to evaluate how well RTAB-Map can localize (only by vision) on maps taken at different illumination conditions. The data has been collected with [RTAB-Map Tango](https://play.google.com/store/apps/details?id=com.introlab.rtabmap&hl=en_CA&gl=US).
![Overview](https://github.com/introlab/rtabmap/raw/master/archive/2022-IlluminationInvariant/images/fig_overview.jpg)


## Description

This folder contains scripts to re-generate results from the paper. The main idea behind this work is that using multi-session mapping can help to localize visually in illumination changing environments even with features that are not very robust to such conditions. We compared common hand-made visual features like SIFT, SURF, BRIEF, BRISK, FREAK, DAISY, KAZE with learned descriptor SuperPoint. The following picture show how robust are the visual features tested when localizing against single session recorded at different time. For example, the bottom-left and top-right cells are when the robot tries to localize the night on a map taken the day or vice-versa. The diagonal is localization performance when the localization session is about the same time than when the map was recorded. SuperPoint has clearly an advantage on this single-session experiment.

![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2022-IlluminationInvariant/images/fig_single_percentage.jpg)

The following image shows when we do the same localization experiment at different hours, but against maps created by assembling maps taken at different hours. In this case, we can see that even binary features like BRIEF can work relatively well in illumination-variant environments. See the paper for more detailled results and comments. The line `1+2+3+4+5+6` refers to the assembled map shown below containing all mapping sessions linked together in same database.

![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2022-IlluminationInvariant/images/fig_merged_percentage.jpg)


![All sessions](https://github.com/introlab/rtabmap/raw/master/archive/2022-IlluminationInvariant/images/fig_map_merged_999.jpg)


## Dataset

We provide two formats: the first one is more general and the second one is used to produce the results in this paper with RTAB-Map. Please open issue if the links are outdated.

* [Images](https://usherbrooke-my.sharepoint.com/:u:/g/personal/labm2414_usherbrooke_ca/EV8F4PZUxOxLhwAyEehlzKwBjF-9xNuxR32Q4mUjx5u-rA?e=eCJ3TW):
  * `rgb`: folder containing *.jpg color camera images
  * `depth`: folder containing *.png 16bits mm depth images
  * `calib`: folder containing calibration for each color image. Each calibration contains also the transform between `device` and `camera` frames as `local_transform`.
  * `device_poses.txt`: VIO poses of each image in `device` frame
  * `camera_poses.txt`: VIO poses of each image in `camera` frame
* [RTAB-Map Databases](https://usherbrooke-my.sharepoint.com/:u:/g/personal/labm2414_usherbrooke_ca/EU5fb0jEKzlGhPK3OWjMGLUBnDo1BRAoZwtB2czyeVLE_A?e=Y0JyXY)



## How reproduce results shown in the paper

1. RTAB-Map should be built from source with those dependencies (don't need to "install" it, we will launch it from build directory in the scripts below to avoid conflicting with another rtabmap already installed): 
    * Use Ubuntu 20.04+ to avoid any python2/python3 conflicts.
    * OpenCV built with **xfeatures2d** and **nonfree** modules
    * [torchlib c++](https://pytorch.org/get-started/locally/) (tested on v1.10.2) to enable [SuperPoint](https://github.com/magicleap/SuperPointPretrainedNetwork)
    * Git clone [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) into scripts directory.
    * Generate `superpoint_v1.pt` in the scripts directory (can also be downloaded from [here](https://github.com/KinglittleQ/SuperPoint_SLAM/blob/master/superpoint.pt) but may not be compatible with more recent pytorch versions):
      ```bash
      cd rtabmap/archive/2022-IlluminationInvariant/scripts
      wget https://github.com/magicleap/SuperPointPretrainedNetwork/raw/master/superpoint_v1.pth
      wget https://raw.githubusercontent.com/magicleap/SuperPointPretrainedNetwork/master/demo_superpoint.py
      python trace.py
      ```

2. Download databases of the dataset and extract them.
3. Adjust the path inside `rtabmap_latest.sh` script to match where you just built rtabmap with right dependencies.
4. Run `run_all.sh DATABASES_PATH OUTPUT_PATH`, this script will do the following steps (warning, this could take hours to do...):
    * Recreate the map databases for each feature type
    * Create the merged databases
    * Run localization databases over all map/merged databases
    * Run consecutive localization experiment
  
5. Export statistics with `export_stats.sh` script.

6. Use the MatLab/Octave scripts in this folder to show results you want. Set `dataDir` to directory containing the exported statistics. 
```
sudo apt install install octave liboctave-dev

# In octave:
pkg install -forge control signal
```

### Docker

1. Create the docker image:
```
cd rtabmap
docker build -t rtabmap_frontiers -f docker/frontiers2022/Dockerfile .
```

2. Assuming you extracted the databases of the dataset in `~/Downloads/Illumination_invariant_databases`, create an output directory for results:
```
mkdir ~/Downloads/Illumination_invariant_databases/results
```

3. Run script:
```
docker run --gpus all -it --rm --ipc=host --runtime=nvidia \
    --user $(id -u):$(id -g) \
    -w=/workspace/scripts \
    -v ~/Downloads/Illumination_invariant_databases:/workspace/databases \
    -v ~/Downloads/Illumination_invariant_databases/results:/workspace/results \
    rtabmap_frontiers /workspace/scripts/run_all.sh /workspace/databases /workspace/results
```

4. Export statistics:
```
docker run --gpus all -it --rm --ipc=host --runtime=nvidia \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --user $(id -u):$(id -g) \
    -w=/workspace/results \
    -v ~/Downloads/Illumination_invariant_databases/results:/workspace/results \
    rtabmap_frontiers /workspace/scripts/export_stats.sh /workspace/results
```
