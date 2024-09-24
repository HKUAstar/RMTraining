`assignment2_pkg` contains submissions to tutorial2: Introduction2ROS
To run the package, follow the steps below: (I write this as a review)
- `cd your_ws_containing_assignment2_pkg`
- `catkin_make`
- `source devel/setup.bash`
- If you wish to run nodes for question1
  - `roslaunch assignment2_pkg question1.launch`
- If you wish to run nodes for question2
  - `export TURTLEBOT3_MODEL=burger`: set an environment variable to run turtlebot3
  - run the given command
  - `rosrun assignment2_pkg points.py`
  - ps `question2.launch` still has fatal errors ...
