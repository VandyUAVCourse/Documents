# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navmap: 5 messages, 0 services")

set(MSG_I_FLAGS "-Inavmap:/home/UAVTeam/catkin_ws/src/navmap/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navmap_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
)
_generate_msg_cpp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
)
_generate_msg_cpp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
)
_generate_msg_cpp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
)
_generate_msg_cpp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
)

### Generating Services

### Generating Module File
_generate_module_cpp(navmap
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navmap_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navmap_generate_messages navmap_generate_messages_cpp)

# target for backward compatibility
add_custom_target(navmap_gencpp)
add_dependencies(navmap_gencpp navmap_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navmap_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
)
_generate_msg_lisp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
)
_generate_msg_lisp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
)
_generate_msg_lisp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
)
_generate_msg_lisp(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
)

### Generating Services

### Generating Module File
_generate_module_lisp(navmap
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(navmap_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(navmap_generate_messages navmap_generate_messages_lisp)

# target for backward compatibility
add_custom_target(navmap_genlisp)
add_dependencies(navmap_genlisp navmap_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navmap_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
)
_generate_msg_py(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
)
_generate_msg_py(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
)
_generate_msg_py(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
)
_generate_msg_py(navmap
  "/home/UAVTeam/catkin_ws/src/navmap/msg/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
)

### Generating Services

### Generating Module File
_generate_module_py(navmap
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navmap_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navmap_generate_messages navmap_generate_messages_py)

# target for backward compatibility
add_custom_target(navmap_genpy)
add_dependencies(navmap_genpy navmap_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navmap_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navmap
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(navmap_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(navmap_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navmap
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(navmap_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(navmap_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navmap
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(navmap_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(navmap_generate_messages_py geometry_msgs_generate_messages_py)
