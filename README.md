#### Terminal 1
$ catkin_make 
$ source devel/setup.bash  
$ roslaunch ur3_driver ur3_gazebo.launch  

#### Terminal 2
$ source devel/setup.bash
$ rosrun lab2pkg_py lab2_spawn_bin.py
$ rosrun lab2pkg_py lab2_spawn_block.py (first start off with reload=no, and if for future loads can set reload=yes)

#### Terminal 3
$ source devel/setup.bash
$ rosrun lab2pkg_py lab2_exec.py  






