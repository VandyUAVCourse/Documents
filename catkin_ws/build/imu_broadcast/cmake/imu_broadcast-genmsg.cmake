# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "imu_broadcast: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iimu_broadcast:/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(imu_broadcast_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_broadcast
)
_generate_msg_cpp(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_broadcast
)

### Generating Services

### Generating Module File
_generate_module_cpp(imu_broadcast
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_broadcast
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(imu_broadcast_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(imu_broadcast_generate_messages imu_broadcast_generate_messages_cpp)

# target for backward compatibility
add_custom_target(imu_broadcast_gencpp)
add_dependencies(imu_broadcast_gencpp imu_broadcast_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_broadcast_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_broadcast
)
_generate_msg_lisp(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_broadcast
)

### Generating Services

### Generating Module File
_generate_module_lisp(imu_broadcast
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_broadcast
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(imu_broadcast_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(imu_broadcast_generate_messages imu_broadcast_generate_messages_lisp)

# target for backward compatibility
add_custom_target(imu_broadcast_genlisp)
add_dependencies(imu_broadcast_genlisp imu_broadcast_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_broadcast_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/raw_imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast
)
_generate_msg_py(imu_broadcast
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast
)

### Generating Services

### Generating Module File
_generate_module_py(imu_broadcast
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(imu_broadcast_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(imu_broadcast_generate_messages imu_broadcast_generate_messages_py)

# target for backward compatibility
add_custom_target(imu_broadcast_genpy)
add_dependencies(imu_broadcast_genpy imu_broadcast_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imu_broadcast_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_broadcast)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imu_broadcast
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_broadcast_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_broadcast)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imu_broadcast
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(imu_broadcast_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imu_broadcast
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(imu_broadcast_generate_messages_py std_msgs_generate_messages_py)
