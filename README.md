# ros_rrt_planner


## How do I get set up?

* trac-ik
``` 
sudo apt-get install ros-kinetic-trac-ik-lib
```

* trajectory lib
```
git clone https://github.com/ggory15/trajectory_smoothing --recursive
cd trajectory_smoothing
mkdir build && cd build
cmake ..
make
sudo make install
```

* rbdl
```
wget https://bitbucket.org/rbdl/rbdl/get/849d2aee8f4c.zip
unzip 849d2aee8f4c.zip
cd rbdl-rbdl-849d2aee8f4c
mkdir build
cd build
cmake -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make all
sudo make install
```

** If an error occurs, open rbdl-rbdl-[commit]/addons/urdfreader/urdfreader.cc and remove this line
```
#include <ros.h>
```


## How do I run the simulation?

* Launch V-Rep after roscore
* Open .ttt
* roslaunch
```
roslaunch ros_rrt_planner demo.launch
```
