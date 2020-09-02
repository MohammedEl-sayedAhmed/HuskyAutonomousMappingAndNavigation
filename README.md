# HuskyAutonomousMappingAndNavigation

In this project, the robot, Husky, is spawned in 5 different environments. In each of
them, the robot uses the gmapping algorithm and autonomously maps the environments using its
laser scanner. Next, the robot is prompted to navigate from its initial position to a final one
across the map. The robot successfully executes those missions; building accurate maps and
navigating using the shortest path.

# For Autonomous Mapping 
Steps:\
1 - roslaunch husky_autonomous_mapping husky_autonomous_mapping_map1.launch\

change [husky_autonomous_mapping_map1.launch] with [xxxxx_mapX] where X = 1,2,3,4,5\



