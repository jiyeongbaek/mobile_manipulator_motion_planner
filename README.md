# ros_rrt_plannar


## How do I get set up?

### trac-ik
``` 
sudo apt-get install ros-kinetic-trac-ik-lib
```

### trajectory lib
```
git clone https://github.com/ggory15/trajectory_smoothing --recursive
cd trajectory_smoothing
mkdir build && cd build
cmake ..
make
sudo make install
'''
