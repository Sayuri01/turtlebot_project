
***************************				
  RUNNING INSTRUCTIONS

***************************
(Done with ROS Kinetic)

Here are the commands to run turtlebot codes for the mapping and objects detection parts together on Gazebo simulation.

*** Requirements ***

Maybe obvious but you have to have Gazebo and Turtlebot 2 packages installed on your PC.

In the .bashrc file, add these lines :
- export GAZEBO_RESOURCE_PATH=/path to your catkin workspace/src/turtlebot_project/worlds/
(It is added to be able to use your own worlds and load textures from the turtlebot_project package)
- export TURTLEBOT_3D_SENSOR="hokuyo"
(It is added to be able to use the turtlebot laser in the Gazebo simulation)


*** Commands ***

1. Start ROS
Terminal 1: roscore

2. Run the simulation world
Terminal 2: roslaunch turtlebot_project turtlebot_rss1.world

3. Run the gmapping 
Terminal 3: roslaunch gmapping_turtlebot gmapping_gazebo_hokuyo.launch

4. Run Rviz to see the turtlebot and the map
Terminal 4: rosrun rviz rviz

5. Run the code to be able to teleoperate the turtlebot
Terminal 5: roslaunch turtlebot_teleop keyboard_teleop.launch

6. Run the code to select images that you want to be detected and to detect them.
Those images are in the turtlebot_object_detection folder in the folder named objects 
Terminal 6: roslaunch find_object_2d find_object_3d_gazebo_with_markers.launch

7. Run the code to detect objects by making the difference between a dead person (number 3) and an alive person (number 7). You can see other object numbers by going in the turtlebot_object_detection folder in the folder named objects. Each image has its specific number.
To run this command, you have to be in your catkin workspace folder.
Terminal 7: rosrun turtlebot_object_detection turtlebotFindObject

8. Run the code to display marker in Rviz where and when an object is detected.
To run this command, you have to be in your catkin workspace folder.
Terminal 8: rosrun turtlebot_object_detection markerPublisher


If errors occur, please do not forget to SOURCE the .bashrc file ;)

