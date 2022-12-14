			+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			++++++++++++++++++++++++++ ADVERTISEMENT ++++++++++++++++++++++++++
			+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

				The set-up of this github page is still work in progress.
				Stay tuned and do not miss further developments.

			+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

***

# UVO - Underwater Visual Odometry
This repository contains the ROS implementation of two Visual Odometry (VO) strategies tailored for the underwater domain developed by the underwater robotics reasearch group of Department of Information Engineering at University of Pisa, which is part of the Interuniversity Center of Integrated Systems for the Marine Environment (ISME). 

## Monocular UVO
This strategy exploits a single camera together with range information to solve the scale ambiguity issue. The strategy is feature-based and switches between essential and homography matrix for relative motion estimation. The estimated motion is used to triangulate matched features, which are then compared to scene distance information (available from an additional sensor) to retrieve the scale factor. 


## Stereo UVO
This solution exploits a stereo VO approach, following a 3D-to-2D method that allows to recover the motion between two consecutive camera views from the visual feedback of a stereo camera.

***

### Input/Output
In both cases, the input is a sequence of distorted and compressed images. The monocular strategy takes as additional input the range observations from a distance sensor, necessary to recover the scale factor at each iteration. 
Instead, the provided output is composed of:
* global position of the vision system, expressed in coordinates with respect to the NED reference system;
* sparse 3D reconstruction of the scene framed by the vision system, in the form of point cloud in coordinates with respect to the NED reference system; 
* the relative motion between previous and current camera views, in terms of angular velocity, linear velocity and euler angles;
* validity flag to assess the successfulness of the estimate.

The ROS diagram of the two UVO nodes including their input and output topics is shown below.

#### <ins>Monocular VO ROS diagram</ins>
<p align="center">
  <img width="560" src="/imgs/monoVO.png">
</p>

#### <ins>Stereo VO ROS diagram</ins>
<p align="center">
  <img width="560" src="/imgs/stereoVO.png">
</p>

### Content
The content of the visual_odometry folder is the following:

* **_config folder_** containing the configuration files that are exploited to set all the parameters of the two nodes. It includes the calibration files and parameter files used within the VO nodes.

* **_include folder_** providing the libraries needed to utilise the two nodes.

* **_launch folder_** with the launch files to use to run the two nodes using ROS framework.

* **_src folder_** containing the nodes implemented for the two VO strategies.

***

## Requirements
UVO has been tested on **Ubuntu 18.04** with **ROS melodic** and **OpenCV 4.5** libraries.

### ROS Melodic
The VO nodes have been developed within ROS framework to share sensor messages between nodes. 

To install the Full-Desktop version of ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu

### OpenCV
OpenCV libraries (http://opencv.org/) are used to elaborate images and estimate camera motion.  

**Note:** OpenCV **extra modules** are required to use *SURF* detector and descriptor. 

Dowload and installation instructions of OpenCV with extra modules can be found at: https://github.com/opencv/opencv_contrib

***

## Execution

Follow this steps to execute the code:

1. Insert the *intrinsic parameters* of your camera within the corresponding configuration file. It should include the radial and tangential *distortion coefficients*.

2. Change the *topic names* and the *type of messages* of the node that you want to execute according to your needs. Alternatively, you can add a node to *remap the topics* in the correct way.

3. Execute the desired *.launch file*.

***

## License & Disclaimer

The UVO source code is released under a GPLv3 license. This is a research code, any fitness for a particular purpose is disclaimed.

***

## Citing

If you use UVO in an academic context, please cite the following publications:

* **Monocular VO**

@article{ruscio2022visual,<br />
&nbsp;&nbsp;  title={Visual-based Navigation Strategy for Autonomous Underwater Vehicles in Monitoring Scenarios},<br />
&nbsp;&nbsp;  author={Ruscio, F and Tani, S and Bresciani, M and Caiti, A and Costanzi, R},<br />
&nbsp;&nbsp;  journal={IFAC-PapersOnLine},<br />
&nbsp;&nbsp;  volume={55},<br />
&nbsp;&nbsp;  number={31},<br />
&nbsp;&nbsp;  pages={369--374},<br />
&nbsp;&nbsp;  year={2022},<br />
&nbsp;&nbsp;  publisher={Elsevier}<br />
}


* **Stereo VO**

@article{tani2022stereo,<br />
&nbsp;&nbsp;  title={Stereo Vision System for Autonomous Ship Hull Inspection},<br />
&nbsp;&nbsp;  author={Tani, S and Ruscio, F and Bresciani, M and Caiti, A and Costanzi, R},<br />
&nbsp;&nbsp;  journal={IFAC-PapersOnLine},<br />
&nbsp;&nbsp;  volume={55},<br />
&nbsp;&nbsp;  number={31},<br />
&nbsp;&nbsp;  pages={375--380},<br />
&nbsp;&nbsp;  year={2022},<br />
&nbsp;&nbsp;  publisher={Elsevier}<br />
}

