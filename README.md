# Cooperative Driving Dataset (CODD)

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5720317.svg)](https://doi.org/10.5281/zenodo.5720317)
[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

The Cooperative Driving dataset is a synthetic dataset generated using [CARLA](https://github.com/carla-simulator/carla) that contains lidar data from multiple vehicles navigating simultaneously through a diverse set of driving scenarios.
This dataset was created to enable further research in multi-agent perception (cooperative perception) including cooperative 3D object detection, cooperative object tracking, multi-agent SLAM and point cloud registration.
Towards that goal, all the frames have been labelled with ground-truth sensor pose and 3D object bounding boxes.

This repository details the organisation of the dataset, including its data sctructure, and how to visualise the data.
Additionally, it contains the code used to create the dataset, allowing users to customly create their own dataset.

![static frame](static/shot.png)
![video showing frames](static/video2.gif)

## Data structure

The dataset is composed of snippets, each containing a sequence of temporal frames in one driving environment. Each frame in a snippet corresponds to a temporal slice of data, containing sensor data (lidar) from **all** vehicles in that environment, as well as the absolute pose of the sensor and ground-truth annotations for the 3D bounding boxes of vehicles and pedestrians.
Each snippet is saved as an HDF5 file containing the following arrays (HDF5 datasets):

- `pointcloud` with dimensions `[frames, vehicles, points_per_cloud, 4]` where the last dimensions represent the X,Y,Z and intensity coordinates of the lidar points in the local sensor coordinate system.
- `lidar_pose` with dimensions `[frames, vehicles, 6]` where the last coordinates represent the X,Y,Z,pitch,yaw,roll of the global sensor pose. These can be used to compute the transformation that maps from the local sensor coordinate system to the global coordinate system.
- `vehicle_boundingbox` with dimensions `[frames, vehicles, 8]` where the last coordinates represent the 3D Bounding Box encoded by X,Y,Z,yaw,pitch,Width,Length,Height. Note that the X,Y,Z correspond to the centre of the 3DBB in the global coordinate system. The roll angle is ignored (roll=0).
- `pedestrian_boundingbox` with dimensions `[frames, pedestrians , 8]` where the last coordinates represent the 3DBB encoded as before.

Where
- `frames` indicate the number of frames in the snippet.
- `vehicles` is the number of vehicles in the environment. Note that all vehicles have lidars that we use to collect data.
- `point_per_cloud` is the maximum number of points per pointcloud. Sometimes a given pointcloud will have less points that this maximum, in that case we pad the entries with zeros to be able to concatenate them into a uniformly sized array.
- `pedestrians` is the number of pedestrians in the environment.

**Notes:**
1. The point clouds are in the local coordinate system of each sensor, where the transformation from local to global coordinate system is computed using `lidar_pose`.
2. Angles are always in degrees.
3. Pose is represented using the UnrealEngine4 left-hand coordinate system. An example to reconstruct a transformation matrix from local -> global is available in `vis.py`, where such matrix is used to aggregate all local lidar point clouds into a global reference system.
4. The vehicle index is shared across `pointcloud`, `lidar_pose` and `vehicle_boundingbox`, i.e. the point cloud at index [frame,i] correspond to the vehicle with bounding box at [frame,i].
5. The vehicle and pedestrian indices are consistent across frames, allowing to determine the track of a given vehicle/pedestrian.
6. All point clouds of a given frame are synchronised in time - they were captured at exactly the same time instant.

## Downloading the Dataset
Although this repository provides the tools to generate your own dataset (see [Generating your own data](#generating-your-own-data)), we have generated an official release of the dataset.

This dataset contains **108 snippets** across **all available CARLA maps**.
The snippets file names encode the properties of the snippets as `m[mapNumber]v[numVehicles]p[numPedestrians]s[seed].hdf5`.

[Download here](https://doi.org/10.5281/zenodo.5720317).

This official dataset was generated with the following settings:
- 5 fps
- 125 frames (corresponding to 25s of simulation time per snippet)
- 50k points per cloud
- 100m lidar range
- 30 burnt frames (discarded frames in the beggining of simulation)
- nvehicles sampled from a binomial distribution with mean 10 and var 5
- npedestrians sampled from a binomial distribution with mean 5 and var 2

## Visualising the snippets

To visualise the data, please install the following dependencies:
- Python 3.x
- h5py
- numpy
- Mayavi >= 4.7.2 

Then run:
```
python vis.py [path_to_snippet]
```

Note that you may want to pause the animation and adjust the view.
The visualisation iteratively goes through all the frames, presenting the fusion of the point cloud from all vehicles transformed to the global coordinate system.
It also shows the ground-truth bounding boxes for vehicles (in green) and pedestrians (in cyan).

![video showing frames](static/video.gif)

## Generating your own data

### Requirements
Before getting started, please install the following dependencies:
- CARLA >= 0.9.10
- Python 3.x
- h5py
- numpy

Note: If the CARLA python package is not available in the python path you need to manually provide the path to the `.egg` file in `fixpath.py`.

### Creating snippets
To generate the data one must firstly start the CARLA simlator:
```
cd CARLA_PATH
./CARLAUE4.sh
```

Then one can create a snippet using
```
python genSnippet.py --map Town03 --fps 5 --frames 50 --burn 30 --nvehicles 10 --npedestrians 3 --range 100 -s test.hdf5
```
This creates a snippet `test.hdf5` in Town03 with a rate of 5 frames per second, saving 50 frames (corresponds to 10s of simulation time) in a scenario with 10 vehicles (we collect lidar data from all of them) and 3 pedestrians.

The `burn` argument is used to discard the first 30 frames since the vehicles will be stopped or slowly moving (due to inertia), so we would get many highly correlated frames without new information.

Note that this script randomly select a location in the map and tries to spawn all the vehicles within `range` meters of this location, which increases the likelihood the vehicles will share their field-of-view (see one another).

The `range` also specifies the maximum range of the lidar sensors.

The `seed` argument defines the RNG seed which allows to reproduce the same scenario (spawn points, trajectories, etc) and change any sensor characteristics across runs.

For more options, such as the number of points per cloud or the number of lidar lasers, or the lower lidar angle, see `python genSnippet.py -h`.

### Creating a collection of snippets
Alternatively, to generate a collection of snippets one can use
```
python genDataset.py N
```
where `N` specifies the number of snippets to generate.
This script randomly selects a map and sample from specific distributions for number of vehicles and pedestrians.
Other options may be individually set-up within the script.

Note: Town06,Town07 and Town10HD need to be installed separately in CARLA, see [here](https://carla.readthedocs.io/en/latest/start_quickstart/#import-additional-assets).

## Citation
If you use our dataset or generate your own dataset using parts of our code, please cite
```
@article{arnold2021fastreg,
  author={Arnold, Eduardo and Mozaffari, Sajjad and Dianati, Mehrdad},
  journal={IEEE Robotics and Automation Letters},
  title={Fast and Robust Registration of Partially Overlapping Point Clouds},
  year={2021},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2021.3137888}
}
```

## License
This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg
