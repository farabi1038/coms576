1. Source your ros environment first using the following command: 

source ~/catkin_ws/devel/setup.bash

2. Do "catkin_make"

3.In one terminal use the following 


 rosrun cs476 hw2_chain_configurator.py \
0.7853981633974483 1.5707963267948966 -0.7853981633974483 -W 2 -L 12 -D 10

4. In another terminal 

source ~/catkin_ws/devel/setup.bash

rosrun cs476 hw2_chain_plotter.py

