FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/dynamixel_hardware_interface/srv/__init__.py"
  "../src/dynamixel_hardware_interface/srv/_StartController.py"
  "../src/dynamixel_hardware_interface/srv/_RestartController.py"
  "../src/dynamixel_hardware_interface/srv/_StopController.py"
  "../src/dynamixel_hardware_interface/srv/_SetComplianceMargin.py"
  "../src/dynamixel_hardware_interface/srv/_SetVelocity.py"
  "../src/dynamixel_hardware_interface/srv/_SetTorqueLimit.py"
  "../src/dynamixel_hardware_interface/srv/_TorqueEnable.py"
  "../src/dynamixel_hardware_interface/srv/_SetComplianceSlope.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
