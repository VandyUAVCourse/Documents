# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pose_estimator: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ipose_estimator:/home/UAVTeam/catkin_ws/src/pose_estimator/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pose_estimator_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/proj_2_5d_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator
)
_generate_msg_cpp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator
)
_generate_msg_cpp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator
)

### Generating Services

### Generating Module File
_generate_module_cpp(pose_estimator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pose_estimator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pose_estimator_generate_messages pose_estimator_generate_messages_cpp)

# target for backward compatibility
add_custom_target(pose_estimator_gencpp)
add_dependencies(pose_estimator_gencpp pose_estimator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimator_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/proj_2_5d_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator
)
_generate_msg_lisp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator
)
_generate_msg_lisp(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator
)

### Generating Services

### Generating Module File
_generate_module_lisp(pose_estimator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pose_estimator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pose_estimator_generate_messages pose_estimator_generate_messages_lisp)

# target for backward compatibility
add_custom_target(pose_estimator_genlisp)
add_dependencies(pose_estimator_genlisp pose_estimator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimator_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/proj_2_5d_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator
)
_generate_msg_py(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/point2d_t.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator
)
_generate_msg_py(pose_estimator
  "/home/UAVTeam/catkin_ws/src/pose_estimator/msg/pose_estimator_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator
)

### Generating Services

### Generating Module File
_generate_module_py(pose_estimator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pose_estimator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pose_estimator_generate_messages pose_estimator_generate_messages_py)

# target for backward compatibility
add_custom_target(pose_estimator_genpy)
add_dependencies(pose_estimator_genpy pose_estimator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pose_estimator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pose_estimator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(pose_estimator_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pose_estimator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(pose_estimator_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pose_estimator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(pose_estimator_generate_messages_py std_msgs_generate_messages_py)
