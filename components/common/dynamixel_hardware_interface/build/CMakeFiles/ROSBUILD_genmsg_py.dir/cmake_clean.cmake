FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/dynamixel_hardware_interface/msg/__init__.py"
  "../src/dynamixel_hardware_interface/msg/_MotorStateList.py"
  "../src/dynamixel_hardware_interface/msg/_JointState.py"
  "../src/dynamixel_hardware_interface/msg/_MotorState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
