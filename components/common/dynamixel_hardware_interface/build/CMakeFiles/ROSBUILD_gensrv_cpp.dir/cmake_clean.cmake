FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/StartController.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/RestartController.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/StopController.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/SetComplianceMargin.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/SetVelocity.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/SetTorqueLimit.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/TorqueEnable.h"
  "../srv_gen/cpp/include/dynamixel_hardware_interface/SetComplianceSlope.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
