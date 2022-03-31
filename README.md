# Convert ROS GPS/GNSS messages to `geometry_msgs/PoseStamped` and `nav_msgs/Path` to visualize in Rviz
+ `sensor_msgs/NavSatFix` message to `geometry_msgs/PoseStamped` and `nav_msgs/Path`

<br>

### Requirement
+ Install [`utm`](https://github.com/Turbo87/utm) python library
~~~bash
$ python -m pip install utm #for python2

or

$ python3 -m pip install utm #for python3

$ cd <your_workspace>/src
$ git clone https://github.com/engcang/utm_to_pose_path.git
$ cd ..
$ catkin build
$ source devel/setup.bash
~~~

### Execution
+ simply run
~~~bash
$ roslaunch utm_to_pose_path utm_to_pose_path.launch
~~~
+ Edit `parent frame id`, `child frame id`, and `topic name` in launch files

+ for visualization,
~~~bash
$ roscd utm_to_pose_path
$ rviz -d rviz.rviz
~~~