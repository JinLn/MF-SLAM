# MF-SLAM

###  Beta version, 21 October 2020
**Authors:** Mingchi Feng, **Jinglin Liu**, Xin Wang, Chengnan Li, [MF-SLAM: Multi-focal SLAM](https://link.springer.com/chapter/10.1007/978-3-030-89134-3_45)
    
*MF-SLAM * is a multi focal length visual slam, which can combine two cameras with different focal lengths to form stereo vision, or two cameras with the same focal length. It is improved on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) . Thank them very much for ORB-SLAM3's work and great efforts.
 
DOI: 10.1007/978-3-030-89134-3_45

We provide examples to run MF-SLAM in the [KITTI](http://www.cvlibs.net/datasets/kitti/) and [self-recorded dataset](添加地址).

This software is based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) developed by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

### Video
[billbill](https://www.bilibili.com/video/BV1FR4y1t7sV/)
[youtube]()

### KITTI dataset experiment
<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/mf00.svg" 
alt="ORB-SLAM2" width="400" height="400" border="0" /></a>

### Self-recorded dataset experiment
<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/mf12_6mm%2612mm.svg" 
alt="6mm&12mm" width="300" height="300" border="0" /><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/mf12_12mm%2616mm.svg" 
alt="12mm&16mm" width="300" height="300" border="0" /><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/mf12_16mm%2625mm.svg" 
alt="16mm&25mm" width="300" height="300" border="0" /></a>

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/playground%20.svg" 
alt="playground" width="300" height="300" border="0" /><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/Academic%20Building.svg" 
alt="Academic_Building" width="300" height="300" border="0" /><img src="https://github.com/JinLn/MF-SLAM/blob/main/Examples/picture/library.svg" 
alt="library" width="300" height="300" border="0" /></a>

# 1. License

MF-SLAM is released under [GPLv3 license](https://https://github.com/JinLn/MF-SLAM/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/JinLn/MF-SLAM/blob/main/Dependencies.md).

For a closed-source version of MF-SLAM for commercial purposes, please contact the authors: **Mingchi Feng**

If you use MF-SLAM in an academic work, please cite:

    @article{ORBSLAM3_2020,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={arXiv preprint arXiv:2007.11898},
      year={2020}
      
      title={{MF-SLAM}: Multi-focal SLAM {SLAM}},
      author={ Mingchi Feng, Jinglin Liu, Xin Wang, Chengnan Li.},
      journal={CIRA 2021: Intelligent Robotics and Applications},
      year={2021}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04** , **18.04** and **18.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 3.0. Tested with OpenCV 3.2.0**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Python
Required to calculate the alignment of the trajectory with the ground truth. **Required Numpy module**.

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) preinstalled with osx

# 3. Building MF-SLAM library and examples

Clone the repository:
```
git clone https://github.com/JinLn/MF-SLAM.git MF-SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *MF-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd MF-SLAM
chmod +x build.sh
./build.sh
```

This will create **libMF-SLAM.so**  at *lib* folder and the executables in *Examples* folder.

# 4. KITTI Examples
[KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) was provide 11 sequences (00-10) with ground truth trajectories for training and 11 sequences (11-21) without ground truth for evaluation. For this benchmark you may provide results using monocular or stereo visual odometry, laser-based SLAM or algorithms that combine visual and LIDAR information. We provide an example script to launch KITTI sequences in all the sensor configurations.

1. Download a sequence from http://www.cvlibs.net/datasets/kitti/eval_odometry.php
 ```
    ./stereo_test ./Vocabulary/ORBvoc.txt /home/jinln/jinln/DATASET/kitti/MF-dyna_yaml/KITTI_stereo_00-02.yaml /home/jinln/jinln/DATASET/kitti/00
```

3. Evaluation using [evo](https://github.com/MichaelGrupp/evo) tools

## Evaluation

```
 evo_traj kitti KITTI_00_ORB.txt MF-SLAM_00.txt --ref=KITTI_00_gt.txt -p --plot_mode=xz
```

# 5. Self-recorded dataset Examples
```
    ./stereo_test ./Vocabulary/ORBvoc.txt /home/jinln/jinln/DATASET/baslerstereodata/20210414/16-25/MF_01.yaml /home/jinln/jinln/DATASET/baslerstereodata/20210414/16-25/2021041406
```
