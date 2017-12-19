# UT BWI GPD Grasp

To launch:
- roslaunch segbot_arm_launch segbot_v2.arm.launch
- roslaunch segbot_arm_launch xtion.launch
- roslaunch gpd tutorial1.launch
- rosrun bwi_grasp convert_grasp
- To run node:
  - rosrun bwi_grasp bwi_grasp_node
- To run action server
  - rosrun bwi_grasp test_gpd_grasp_action_node
  - rosrun bwi_grasp gpd_grasp_action

The purpose of this package: We would like to change the BWIBot's arm to use gpd instead of agile grasp. 
