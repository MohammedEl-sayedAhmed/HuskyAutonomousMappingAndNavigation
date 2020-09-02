# HuskyAutonomousMappingAndNavigation

In this project, the robot, Husky, is spawned in 5 different environments. In each of
them, the robot uses the gmapping algorithm and autonomously maps the environments using its
laser scanner. Next, the robot is prompted to navigate from its initial position to a final one
across the map. The robot successfully executes those missions; building accurate maps and
navigating using the shortest path.

# For Autonomous Mapping; run the following
Steps:\
1 - roslaunch husky_autonomous_mapping husky_autonomous_mapping_map1.launch

change [husky_autonomous_mapping_map1.launch] with [xxxxx_mapX.launch] where X = 1,2,3,4,5

# For saving the generated map; run the following
Steps:\
1 - rosrun map_server map_saver -f ~/generated_map

# For navigating in different worlds; follow the follwing steps
Steps:\
1 - Open "chefbot/chefbot_gazebo/launch/amcl_demo_map1.launch"<br />
2 - Change the default value of the map file name to the name of the desired map and its path<br />
3 - Run roslaunch husky_navigating husky_navigating_map1.launch<br />
change [husky_navigating husky_navigating_map1.launch] with [xxxxx_mapX.launch] where X = 1,2,3,4,5


