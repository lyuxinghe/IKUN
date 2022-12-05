#### Terminal 1
$ catkin_make 
$ source devel/setup.bash  
$ roslaunch ur3_driver ur3_gazebo.launch  

#### Terminal 2
$ source devel/setup.bash
$ rosrun lab2pkg_py lab2_spawn.py
(you can input arbitrary values for block location/ missing, since they do nothing for now)

#### Terminal 3
$ source devel/setup.bash
$ rosrun lab2pkg_py lab2_exec.py  






