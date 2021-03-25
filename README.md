# MIE443 Contest 2 Code

Group 22:
Savo Bajic - 1003051485
Maximilian Glidden - 1002277396
Catherine Kucaba - 1003278026

## Commands needed to properly execute the robot code
Command lines required to initiate the code are given in sequential order below. They are no different to the standard set in the contest outlines. 
NOTE: Each one of these lines need to be executed in seperate terminal windows, in the order presented below. 
NOTE: the file path at the end of the third command "roslaunch turtlebot_..." needs to be adjusted for the testing computer so it points to the right map file for the contest.

clear; cd ~/catkin_ws/; catkin_make

roslaunch mie443_contest2 turtlebot_world.launch world:=practice

roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/brobot/catkin_ws/src/mie443_contest2/maps/map_practice.yaml

rosrun mie443_contest2 contest2

If a visualization of the state of the robot's localization is desired during the run, execute the following command in a seperate terminal either right before "rosrun mie443_contest2 contest2" or after it.

roslaunch turtlebot_rviz_launchers view_navigation.launch

## Commands required after the code executes
There are no commands needed once execution is complete. However the results can be quickly reached and read in terminal by issuing the following command:

nano ~/Documents/team22results.txt 

## Computer specific file locations in our code
There are no computer specific file locations that need to be modified in our code if our diectory has been downloaded in its entirety to the user's catkin workspace properly. 

However there is the map string mentioned when initiating the robot's code.

## Output file
The output file will be stored in the documents folder ($HOME/Documents/, typically ~/Documents/) as "team22result.txt", for an overall location of:

~/Documents/team22results.txt
