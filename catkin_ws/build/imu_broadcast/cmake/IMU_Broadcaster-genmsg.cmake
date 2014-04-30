# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "IMU_Broadcaster: 1 messages, 0 services")

set(MSG_I_FLAGS "-IIMU_Broadcaster:/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(IMU_Broadcaster_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(IMU_Broadcaster
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/IMU_Broadcaster
)

### Generating Services

### Generating Module File
_generate_module_cpp(IMU_Broadcaster
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/IMU_Broadcaster
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(IMU_Broadcaster_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(IMU_Broadcaster_generate_messages IMU_Broadcaster_generate_messages_cpp)

# target for backward compatibility
add_custom_target(IMU_Broadcaster_gencpp)
add_dependencies(IMU_Broadcaster_gencpp IMU_Broadcaster_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS IMU_Broadcaster_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(IMU_Broadcaster
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/IMU_Broadcaster
)

### Generating Services

### Generating Module File
_generate_module_lisp(IMU_Broadcaster
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/IMU_Broadcaster
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(IMU_Broadcaster_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(IMU_Broadcaster_generate_messages IMU_Broadcaster_generate_messages_lisp)

# target for backward compatibility
add_custom_target(IMU_Broadcaster_genlisp)
add_dependencies(IMU_Broadcaster_genlisp IMU_Broadcaster_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS IMU_Broadcaster_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(IMU_Broadcaster
  "/home/UAVTeam/catkin_ws/src/imu_broadcast/msgs/attitude.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/IMU_Broadcaster
)

### Generating Services

### Generating Module File
_generate_module_py(IMU_Broadcaster
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/IMU_Broadcaster
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(IMU_Broadcaster_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(IMU_Broadcaster_generate_messages IMU_Broadcaster_generate_messages_py)

# target for backward compatibility
add_custom_target(IMU_Broadcaster_genpy)
add_dependencies(IMU_Broadcaster_genpy IMU_Broadcaster_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS IMU_Broadcaster_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/IMU_Broadcaster)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/IMU_Broadcaster
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(IMU_Broadcaster_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/IMU_Broadcaster)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/IMU_Broadcaster
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(IMU_Broadcaster_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/IMU_Broadcaster)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/IMU_Broadcaster\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/IMU_Broadcaster
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(IMU_Broadcaster_generate_messages_py std_msgs_generate_messages_py)
