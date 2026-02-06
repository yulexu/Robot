# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "feetech_controls: 3 messages, 2 services")

set(MSG_I_FLAGS "-Ifeetech_controls:/home/linzp/catkin_op/src/feetech_controls/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(feetech_controls_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_custom_target(_feetech_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feetech_controls" "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" ""
)

get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_custom_target(_feetech_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feetech_controls" "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" ""
)

get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_custom_target(_feetech_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feetech_controls" "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" ""
)

get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_custom_target(_feetech_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feetech_controls" "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" ""
)

get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_custom_target(_feetech_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "feetech_controls" "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
)
_generate_msg_cpp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
)
_generate_msg_cpp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
)

### Generating Services
_generate_srv_cpp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
)
_generate_srv_cpp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
)

### Generating Module File
_generate_module_cpp(feetech_controls
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(feetech_controls_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(feetech_controls_generate_messages feetech_controls_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_cpp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_cpp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_cpp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_cpp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_cpp _feetech_controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feetech_controls_gencpp)
add_dependencies(feetech_controls_gencpp feetech_controls_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feetech_controls_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
)
_generate_msg_eus(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
)
_generate_msg_eus(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
)

### Generating Services
_generate_srv_eus(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
)
_generate_srv_eus(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
)

### Generating Module File
_generate_module_eus(feetech_controls
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(feetech_controls_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(feetech_controls_generate_messages feetech_controls_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_eus _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_eus _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_eus _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_eus _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_eus _feetech_controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feetech_controls_geneus)
add_dependencies(feetech_controls_geneus feetech_controls_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feetech_controls_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
)
_generate_msg_lisp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
)
_generate_msg_lisp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
)

### Generating Services
_generate_srv_lisp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
)
_generate_srv_lisp(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
)

### Generating Module File
_generate_module_lisp(feetech_controls
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(feetech_controls_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(feetech_controls_generate_messages feetech_controls_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_lisp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_lisp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_lisp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_lisp _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_lisp _feetech_controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feetech_controls_genlisp)
add_dependencies(feetech_controls_genlisp feetech_controls_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feetech_controls_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
)
_generate_msg_nodejs(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
)
_generate_msg_nodejs(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
)

### Generating Services
_generate_srv_nodejs(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
)
_generate_srv_nodejs(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
)

### Generating Module File
_generate_module_nodejs(feetech_controls
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(feetech_controls_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(feetech_controls_generate_messages feetech_controls_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_nodejs _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_nodejs _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_nodejs _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_nodejs _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_nodejs _feetech_controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feetech_controls_gennodejs)
add_dependencies(feetech_controls_gennodejs feetech_controls_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feetech_controls_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
)
_generate_msg_py(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
)
_generate_msg_py(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
)

### Generating Services
_generate_srv_py(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
)
_generate_srv_py(feetech_controls
  "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
)

### Generating Module File
_generate_module_py(feetech_controls
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(feetech_controls_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(feetech_controls_generate_messages feetech_controls_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_py _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/jointfeedback.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_py _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/msg/headcontrols.msg" NAME_WE)
add_dependencies(feetech_controls_generate_messages_py _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/torqcontrol.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_py _feetech_controls_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/linzp/catkin_op/src/feetech_controls/srv/readjointvalues.srv" NAME_WE)
add_dependencies(feetech_controls_generate_messages_py _feetech_controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(feetech_controls_genpy)
add_dependencies(feetech_controls_genpy feetech_controls_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS feetech_controls_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/feetech_controls
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(feetech_controls_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/feetech_controls
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(feetech_controls_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/feetech_controls
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(feetech_controls_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/feetech_controls
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(feetech_controls_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/feetech_controls
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(feetech_controls_generate_messages_py std_msgs_generate_messages_py)
endif()
