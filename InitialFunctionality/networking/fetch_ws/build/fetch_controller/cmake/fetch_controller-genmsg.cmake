# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fetch_controller: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifetch_controller:/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fetch_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_custom_target(_fetch_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fetch_controller" "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fetch_controller
  "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_controller
)

### Generating Services

### Generating Module File
_generate_module_cpp(fetch_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fetch_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fetch_controller_generate_messages fetch_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_dependencies(fetch_controller_generate_messages_cpp _fetch_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_controller_gencpp)
add_dependencies(fetch_controller_gencpp fetch_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fetch_controller
  "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_controller
)

### Generating Services

### Generating Module File
_generate_module_eus(fetch_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fetch_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fetch_controller_generate_messages fetch_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_dependencies(fetch_controller_generate_messages_eus _fetch_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_controller_geneus)
add_dependencies(fetch_controller_geneus fetch_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fetch_controller
  "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_controller
)

### Generating Services

### Generating Module File
_generate_module_lisp(fetch_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fetch_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fetch_controller_generate_messages fetch_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_dependencies(fetch_controller_generate_messages_lisp _fetch_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_controller_genlisp)
add_dependencies(fetch_controller_genlisp fetch_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fetch_controller
  "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_controller
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fetch_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fetch_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fetch_controller_generate_messages fetch_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_dependencies(fetch_controller_generate_messages_nodejs _fetch_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_controller_gennodejs)
add_dependencies(fetch_controller_gennodejs fetch_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fetch_controller
  "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_controller
)

### Generating Services

### Generating Module File
_generate_module_py(fetch_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fetch_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fetch_controller_generate_messages fetch_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ubuntu/Documents/fetch-project-ec545/InitialFunctionality/networking/fetch_ws/src/fetch_controller/msg/controller_state.msg" NAME_WE)
add_dependencies(fetch_controller_generate_messages_py _fetch_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fetch_controller_genpy)
add_dependencies(fetch_controller_genpy fetch_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fetch_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fetch_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(fetch_controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fetch_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(fetch_controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fetch_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(fetch_controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fetch_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(fetch_controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fetch_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(fetch_controller_generate_messages_py std_msgs_generate_messages_py)
endif()
