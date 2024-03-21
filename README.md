# 3D Gaussian Splatting Real-time Renderer from VINS Pose

<table>
  <tr>
     <td> <img src="./pipeline.png" width="500" height="500"> </td>     
  </tr> 
</table>

## Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2 **Catkin Tools**
Install catkin_tools. Follow [catkin_tools Installation](https://catkin-tools.readthedocs.io/en/latest/installing.html)

### 1.3 **Anaconda**
Install Latest Version.
Follow [Anaconda Installation](https://www.anaconda.com/products/distribution#linux)

### 1.4. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## Build 
Clone the repository and catkin_make:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/SungJaeShin/Gaussian_Splatting_Realtime_Render.git
    $ cd ../
    $ catkin build camera_models
    $ source ~/catkin_ws/devel/setup.bash
    $ catkin build vins
    $ source ~/catkin_ws/devel/setup.bash
```

## Run
**You must have a pre-trained model and have bag data and camera parameters !!!**
```
    # Terminal 1
    $ rosrun vins vins_node ./VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

    # Terminal 2
    $ python3 realtime_render.py -m <TRAINED_MODEL_PATH> -s <DATASET_PATH>
 
```


## Results (KAIST N25 Building Indoor Dataset) 
- Random Pose from VINS-Fusion Keyframe Pose \
    ![random_pose](https://github.com/SungJaeShin/Gaussian_Splatting_Realtime_Render/assets/67855888/50135cfc-2a12-4efd-adac-c751887e495e)

- Fixed z position from VINS-Fusion Keyframe Pose \
    ![fixed_z](https://github.com/SungJaeShin/Gaussian_Splatting_Realtime_Render/assets/67855888/ada854f3-091d-4a86-ba31-fbf7ea8ffdc1)


## Reference 
[1] [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git) \
[2] [Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting.git) 

