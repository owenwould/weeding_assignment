# weeding_assignment

weeding_assignment package - PERCEPTION SPECIALISM
- https://github.com/owenwould/weeding_assignment
- git clone https://github.com/owenwould/weeding_assignment.git

Summary of system 

The solution’s chosen focus area is perception, the vision system uses a custom YOLOv3 detector which can quickly and accurately detect weeds and crops. For navigation the solution uses move_base and topological navigation utilising actions. Topological navigation is used to move along a network of nodes which are position at the beginning, middle and end of the cropped rows. Once at a topological node the robot will stop and detect the weeds. The weeds positions are calculated using the PinHole Model from image_geometry and localisation is done using fake localisation. Thorvald will move to these weeds and call the sprayer.


Evaluation Material can be found inside Evaluation folder of package 

Package requires darknet_ros and CMP9767M 
- Ubuntu 18.04, ROS Melodic, Python 2
- get darknet_ros by git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
- get CMP9767M by git clone https://github.com/LCAS/CMP9767M.git

Following Steps to set up Project for use split into 3 sections:
- First section setting up YOLOv3 
- Second section setting up topological map 
- Third section setting move_base config files 
- Final section running project

#First Section 
- After cloning darknet_ros into src folder of catkin workspace

- Go into darknet_ros/darknet_ros/yolo_network_config then in the weights folder place yolov3_custom_tiny.weights into folder which can be found under config/YOLO Config inside weeding assignment 

- After place yolov3_custom_tiny.cfg into folder inside of darknet_ros/darknet_ros/yolo_network_config folder/cfg folder which can be found under config/YOLO Config inside weeding assignment 

- Now Replace ros.yaml inside darknet_ros/darknet_ros/config with ros.yaml found under config/YOLO Config inside weeding assignment 

- Now go place yolov3_custom.yaml inside darknet_ros/darknet_ros/config (same folder as last step) which can be found under config/YOLO Config inside weeding assignment 

- Next go to darknet_ros/darknet_ros/launch and replace both darknet_ros.launch and yolov3.launch with files of the exact same name from config/YOLO Config inside weeding assignment 

- catkin_make then use roslaunch darknet_ros darknet_ros.launch to check if everything set up correctly



#Second Section 
- after installing CMP9767M 

- go to CMP9767M/uol_cmp9767m_tutorial/maps then copy top_map.yaml (from maps folder inside weed assignment package) into this folder

- create a folder called mongodb in home directory 

- run simulation roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true fake_localisation:=true

- run move_base_topo_nav roslaunch uol_cmp9767m_tutorial move_base_topo_nav.launch

- run topo_nav roslaunch uol_cmp9767m_tutorial topo_nav.launch

- when error for missing point set appears run rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/top_map.yaml

- map should be successfully inserted, if it says map is already inserted go inside mongodb and clear files then repeat steps 

- view map by running rviz -d $(rospack find uol_cmp9767m_tutorial)/config/topo_nav.rviz


#Third Step 
- once install CMP9767M
- Replace planners.yaml inside CMP9767M/uol_cmp9767m_tutorial/config with planners.yaml from config inside weeding assignment package
- Replace planners_topo_nav.yaml inside CMP9767M/uol_cmp9767m_tutorial/config with planners_topo_nav.yaml from config inside weeding assignment package


#Final Section how to run weeding package
- run simulator roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true fake_localisation:=true 
- run roslaunch uol_cmp9767m_tutorial move_base_topo_nav.launch 
- run roslaunch uol_cmp9767m_tutorial topo_nav.launch
- run roslaunch darknet_ros darknet_ros.launch 
- Once darknet has loaded image 
- Finally run roslaunch weeding_assignment weeding_launch.launch and the weeding will begin 
- if any edges are single direction or a node is missing then reinsert map, if simulation is shut down whilst topological nav is running 
this can happen




