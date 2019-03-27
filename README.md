# Light VO
A light version SLAM could run with IoT devices

---

### Prerequisites

* ROS kinetic
* OpenCV 3.0+
* In Real-time version: any monocular camera
* In Dataset version: KITTI sequence 00
---

### How to install 
1. Create a ROS workspace
```
mkdir -p slam_ws/src
cd slam/src
catkin_init_workspace
```

2. Clone the repository:
```
git clone https://github.com/Jing-lun/LightSLAM.git
```

3. Build
```
cd ..
catkin_make -j4
```
---

### How to run in real time
1 Install your own camear's driver in ros 

2 Calibrate your camera

3 
```
cd slam_ws/src/light_slam/launch
gedit VO.launch
```   

4 Change the topic and calibration path to your own version

5
  ```
  cd slam_wa
  source devel/setup.bash
  roslauch light_slam VO.launch
  ```
  
6 Don't forget to open your camera's node! Here's my camera noed:
  ```
  roslaunch pointgrey_camera_driver camera.launch
  ```
  
### How to run with dataset (especially for mono-slam)
1 Change the dataset path in src/test_dataset.cc and run_dataset.sh

2 Rebuild && Run it
