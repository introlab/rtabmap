Docker image used to reproduce all results for KITTI, EuRoC and TUM datasets of the following paper (for MIT Stata Center dataset, see this [page](https://github.com/introlab/rtabmap_ros/blob/master/launch/jfr2018)):

 * M. Labbé and F. Michaud, “RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation,” in Journal of Field Robotics, accepted, 2018. ([pdf](https://introlab.3it.usherbrooke.ca/mediawiki-introlab/images/7/7a/Labbe18JFR_preprint.pdf)) ([Wiley](https://doi.org/10.1002/rob.21831))

Pull image:
```bash
$ docker pull introlab3it/rtabmap:jfr2018
```
or create the image with the `Dockerfile` provided in this directory:
```bash
$ cd rtabmap/docker/jfr2018
$ docker build -t introlab3it/rtabmap:jfr2018 . 
```

Make sure to extract datasets in a subfolder called `datasets` relative to scripts in this folder (you can copy the scripts outside rtabmap source directory for convenience). The testing folder tree should look like this:
```bash
datasets/kitti/devkit
datasets/kitti/dataset/sequences/00
datasets/kitti/dataset/sequences/01
datasets/kitti/dataset/sequences/...
datasets/euroc/MH_01_easy
datasets/euroc/V1_01_easy
datasets/euroc/...
datasets/tum/rgbd_dataset_freiburg1_desk
datasets/tum/rgbd_dataset_freiburg3_long_office_household
datasets/tum/...

run_all.sh
run_kitti_datasets.sh
run_euroc_datasets.sh
run_tum_datasets.sh
```

For TUM dataset, use this script [associate.py](https://gist.github.com/matlabbe/484134a2d9da8ad425362c6669824798) to synchronize RGB and depth images before processing. Usage in a TUM dataset: `python associate.py rgb.txt depth.txt`, this will create `rgb_sync` and `depth_sync` folders.

Process all datasets, results will be written to subfolders `results/euroc`, `results/kitti` and `results/tum`:
```bash
./run_all.sh
```
WARNING: processing all datasets with all different odometry approaches can require more than 10 hours to process. You can comment some configurations in [run_all.sh](https://github.com/introlab/rtabmap/blob/master/docker/jfr2018/run_all.sh) if you are interested in just one odometry approach or one dataset. A single sequence can be tested like this too:
```bash
# Processing only sequence 07 of the kitti dataset with F2M odometry approach
./run_kitti_datasets.sh f2m 0 0 07
```

Show all results:
```bash
$ rtabmap-report --scale results
```
