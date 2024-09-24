`assignment2_pkg` contains submissions to tutorial2: Introduction2ROS
To run the package, follow the steps below: (I write this as a review)
- `cd your_ws_containing_assignment2_pkg`
- `catkin_make`
- `source devel/setup.bash`
- If you wish to run nodes for question1
  - `roslaunch assignment2_pkg question1.launch`
- If you wish to run nodes for question2
  - Set the `TURTLEBOT3_MODEL` in ~/.bashrc
  - `source ~/.bashrc` or re-start the terminal
  - `roslaunch assignment2_pkg question2.launch`
