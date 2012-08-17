FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/StartController.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_StartController.lisp"
  "../srv_gen/lisp/RestartController.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_RestartController.lisp"
  "../srv_gen/lisp/StopController.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_StopController.lisp"
  "../srv_gen/lisp/SetComplianceMargin.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetComplianceMargin.lisp"
  "../srv_gen/lisp/SetVelocity.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetVelocity.lisp"
  "../srv_gen/lisp/SetTorqueLimit.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetTorqueLimit.lisp"
  "../srv_gen/lisp/TorqueEnable.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_TorqueEnable.lisp"
  "../srv_gen/lisp/SetComplianceSlope.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SetComplianceSlope.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
