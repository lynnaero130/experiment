# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "offb_posctl: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ioffb_posctl:/home/uav/lzy_ws/src/offb_posctl/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(offb_posctl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_custom_target(_offb_posctl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "offb_posctl" "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(offb_posctl
  "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offb_posctl
)

### Generating Services

### Generating Module File
_generate_module_cpp(offb_posctl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offb_posctl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(offb_posctl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(offb_posctl_generate_messages offb_posctl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_dependencies(offb_posctl_generate_messages_cpp _offb_posctl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offb_posctl_gencpp)
add_dependencies(offb_posctl_gencpp offb_posctl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offb_posctl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(offb_posctl
  "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offb_posctl
)

### Generating Services

### Generating Module File
_generate_module_eus(offb_posctl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offb_posctl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(offb_posctl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(offb_posctl_generate_messages offb_posctl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_dependencies(offb_posctl_generate_messages_eus _offb_posctl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offb_posctl_geneus)
add_dependencies(offb_posctl_geneus offb_posctl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offb_posctl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(offb_posctl
  "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offb_posctl
)

### Generating Services

### Generating Module File
_generate_module_lisp(offb_posctl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offb_posctl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(offb_posctl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(offb_posctl_generate_messages offb_posctl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_dependencies(offb_posctl_generate_messages_lisp _offb_posctl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offb_posctl_genlisp)
add_dependencies(offb_posctl_genlisp offb_posctl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offb_posctl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(offb_posctl
  "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offb_posctl
)

### Generating Services

### Generating Module File
_generate_module_nodejs(offb_posctl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offb_posctl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(offb_posctl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(offb_posctl_generate_messages offb_posctl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_dependencies(offb_posctl_generate_messages_nodejs _offb_posctl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offb_posctl_gennodejs)
add_dependencies(offb_posctl_gennodejs offb_posctl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offb_posctl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(offb_posctl
  "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offb_posctl
)

### Generating Services

### Generating Module File
_generate_module_py(offb_posctl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offb_posctl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(offb_posctl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(offb_posctl_generate_messages offb_posctl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/lzy_ws/src/offb_posctl/msg/controlstate.msg" NAME_WE)
add_dependencies(offb_posctl_generate_messages_py _offb_posctl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(offb_posctl_genpy)
add_dependencies(offb_posctl_genpy offb_posctl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS offb_posctl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offb_posctl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/offb_posctl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(offb_posctl_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offb_posctl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/offb_posctl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(offb_posctl_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offb_posctl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/offb_posctl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(offb_posctl_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offb_posctl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/offb_posctl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(offb_posctl_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offb_posctl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offb_posctl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/offb_posctl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(offb_posctl_generate_messages_py geometry_msgs_generate_messages_py)
endif()
