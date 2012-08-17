FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/dynamixel_hardware_interface/msg"
  "../src/dynamixel_hardware_interface/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MotorStateList.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MotorStateList.lisp"
  "../msg_gen/lisp/JointState.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_JointState.lisp"
  "../msg_gen/lisp/MotorState.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MotorState.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
