FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/dynamixel_controllers.dir/src/joint_position_controller.o"
  "CMakeFiles/dynamixel_controllers.dir/src/joint_torque_controller.o"
  "CMakeFiles/dynamixel_controllers.dir/src/joint_trajectory_action_controller.o"
  "../lib/libdynamixel_controllers.pdb"
  "../lib/libdynamixel_controllers.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/dynamixel_controllers.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
