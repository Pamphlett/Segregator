# ***Segreagator***: Global Point Cloud Registration with Semantic and Geometric Cues

We present Segregator, a global point cloud registration pipeline using both semantic and geometric information.

![](assets/segregator_intro.png)

### Test Environment
* Linux 18.04 LTS
* ROS Melodic

### How to Build
Run the following lines for denpandencies:
```
sudo apt install cmake libeigen3-dev libboost-all-dev
```
Then create a catkin workspace and clone the repo:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:Pamphlett/Segreagator.git
cd Segreagator && mkdir build && cd build
cmake ..
mv pmc-src/ ../../../build/
cd ~/catkin_ws
catkin build segregator 
```

### How to Run
With the provided scans in the ```materials``` folder, run the following lines in the catkin workspace for a toy example:
```
source devel/setup.bash
roslaunch segregator run_segregator.launch
```