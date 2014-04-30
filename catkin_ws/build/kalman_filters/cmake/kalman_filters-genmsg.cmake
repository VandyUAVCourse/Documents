# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kalman_filters: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ikalman_filters:/home/UAVTeam/catkin_ws/src/kalman_filters/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kalman_filters_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_filters
)
_generate_msg_cpp(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_filters
)

### Generating Services

### Generating Module File
_generate_module_cpp(kalman_filters
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_filters
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kalman_filters_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kalman_filters_generate_messages kalman_filters_generate_messages_cpp)

# target for backward compatibility
add_custom_target(kalman_filters_gencpp)
add_dependencies(kalman_filters_gencpp kalman_filters_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_filters_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_filters
)
_generate_msg_lisp(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_filters
)

### Generating Services

### Generating Module File
_generate_module_lisp(kalman_filters
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_filters
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kalman_filters_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kalman_filters_generate_messages kalman_filters_generate_messages_lisp)

# target for backward compatibility
add_custom_target(kalman_filters_genlisp)
add_dependencies(kalman_filters_genlisp kalman_filters_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_filters_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/state_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters
)
_generate_msg_py(kalman_filters
  "/home/UAVTeam/catkin_ws/src/kalman_filters/msg/pose_ukf_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters
)

### Generating Services

### Generating Module File
_generate_module_py(kalman_filters
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kalman_filters_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kalman_filters_generate_messages kalman_filters_generate_messages_py)

# target for backward compatibility
add_custom_target(kalman_filters_genpy)
add_dependencies(kalman_filters_genpy kalman_filters_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kalman_filters_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_filters)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kalman_filters
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_filters_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(kalman_filters_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_filters)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kalman_filters
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_filters_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(kalman_filters_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kalman_filters
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(kalman_filters_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(kalman_filters_generate_messages_py geometry_msgs_generate_messages_py)
