# ***Segreagator***: Global Point Cloud Registration with Semantic and Geometric Cues

We present Segregator, a global point cloud registration pipeline using both semantic and geometric information.

![](assets/segregator_intro.png)

### Test Environment
* Linux 18.04/20.04 LTS
* ROS Melodic/Noetic

### How to Build
Run the following lines for denpandencies:
```
sudo apt install cmake libeigen3-dev libboost-all-dev
```
We use [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) to build the project:
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

### Acknowledgements
We would like to thank [Quatro](https://github.com/url-kaist/Quatro), [Teaser](https://github.com/MIT-SPARK/TEASER-plusplus) as well as [T-LOAM](https://github.com/zpw6106/tloam) for making their project public.