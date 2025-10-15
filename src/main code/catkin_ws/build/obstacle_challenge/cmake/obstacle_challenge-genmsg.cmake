# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "obstacle_challenge: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iobstacle_challenge:/home/wro25/catkin_ws/src/obstacle_challenge/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(obstacle_challenge_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_custom_target(_obstacle_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "obstacle_challenge" "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" ""
)

get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_custom_target(_obstacle_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "obstacle_challenge" "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" ""
)

get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_custom_target(_obstacle_challenge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "obstacle_challenge" "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" "obstacle_challenge/Pillar"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge
)
_generate_msg_cpp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge
)

### Generating Services
_generate_srv_cpp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv"
  "${MSG_I_FLAGS}"
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge
)

### Generating Module File
_generate_module_cpp(obstacle_challenge
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(obstacle_challenge_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(obstacle_challenge_generate_messages obstacle_challenge_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_cpp _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_cpp _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_cpp _obstacle_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacle_challenge_gencpp)
add_dependencies(obstacle_challenge_gencpp obstacle_challenge_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacle_challenge_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge
)
_generate_msg_eus(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge
)

### Generating Services
_generate_srv_eus(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv"
  "${MSG_I_FLAGS}"
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge
)

### Generating Module File
_generate_module_eus(obstacle_challenge
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(obstacle_challenge_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(obstacle_challenge_generate_messages obstacle_challenge_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_eus _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_eus _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_eus _obstacle_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacle_challenge_geneus)
add_dependencies(obstacle_challenge_geneus obstacle_challenge_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacle_challenge_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge
)
_generate_msg_lisp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge
)

### Generating Services
_generate_srv_lisp(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv"
  "${MSG_I_FLAGS}"
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge
)

### Generating Module File
_generate_module_lisp(obstacle_challenge
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(obstacle_challenge_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(obstacle_challenge_generate_messages obstacle_challenge_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_lisp _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_lisp _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_lisp _obstacle_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacle_challenge_genlisp)
add_dependencies(obstacle_challenge_genlisp obstacle_challenge_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacle_challenge_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge
)
_generate_msg_nodejs(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge
)

### Generating Services
_generate_srv_nodejs(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv"
  "${MSG_I_FLAGS}"
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge
)

### Generating Module File
_generate_module_nodejs(obstacle_challenge
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(obstacle_challenge_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(obstacle_challenge_generate_messages obstacle_challenge_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_nodejs _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_nodejs _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_nodejs _obstacle_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacle_challenge_gennodejs)
add_dependencies(obstacle_challenge_gennodejs obstacle_challenge_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacle_challenge_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge
)
_generate_msg_py(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge
)

### Generating Services
_generate_srv_py(obstacle_challenge
  "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv"
  "${MSG_I_FLAGS}"
  "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge
)

### Generating Module File
_generate_module_py(obstacle_challenge
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(obstacle_challenge_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(obstacle_challenge_generate_messages obstacle_challenge_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/ultraInfo.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_py _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/msg/Pillar.msg" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_py _obstacle_challenge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wro25/catkin_ws/src/obstacle_challenge/srv/PillarDetection.srv" NAME_WE)
add_dependencies(obstacle_challenge_generate_messages_py _obstacle_challenge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacle_challenge_genpy)
add_dependencies(obstacle_challenge_genpy obstacle_challenge_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacle_challenge_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacle_challenge
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(obstacle_challenge_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacle_challenge
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(obstacle_challenge_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacle_challenge
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(obstacle_challenge_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacle_challenge
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(obstacle_challenge_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacle_challenge
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(obstacle_challenge_generate_messages_py std_msgs_generate_messages_py)
endif()
