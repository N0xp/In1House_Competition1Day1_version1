# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vmxpi_ros: 3 messages, 7 services")

set(MSG_I_FLAGS "-Ivmxpi_ros:/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vmxpi_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" ""
)

get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_custom_target(_vmxpi_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vmxpi_ros" "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)

### Generating Services
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_cpp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
)

### Generating Module File
_generate_module_cpp(vmxpi_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vmxpi_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vmxpi_ros_generate_messages vmxpi_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_cpp _vmxpi_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vmxpi_ros_gencpp)
add_dependencies(vmxpi_ros_gencpp vmxpi_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vmxpi_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)

### Generating Services
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_eus(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
)

### Generating Module File
_generate_module_eus(vmxpi_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vmxpi_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vmxpi_ros_generate_messages vmxpi_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_eus _vmxpi_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vmxpi_ros_geneus)
add_dependencies(vmxpi_ros_geneus vmxpi_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vmxpi_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)

### Generating Services
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_lisp(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
)

### Generating Module File
_generate_module_lisp(vmxpi_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vmxpi_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vmxpi_ros_generate_messages vmxpi_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_lisp _vmxpi_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vmxpi_ros_genlisp)
add_dependencies(vmxpi_ros_genlisp vmxpi_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vmxpi_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)

### Generating Services
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_nodejs(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
)

### Generating Module File
_generate_module_nodejs(vmxpi_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(vmxpi_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(vmxpi_ros_generate_messages vmxpi_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_nodejs _vmxpi_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vmxpi_ros_gennodejs)
add_dependencies(vmxpi_ros_gennodejs vmxpi_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vmxpi_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_msg_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)

### Generating Services
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)
_generate_srv_py(vmxpi_ros
  "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
)

### Generating Module File
_generate_module_py(vmxpi_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vmxpi_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vmxpi_ros_generate_messages vmxpi_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/LimitSwitch.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/TitanInfo.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/msg/UniqueID.msg" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Int.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/IntRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/Float.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/FloatRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/MotorSpeed.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StopMode.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/pi/catkin_ws/src/vmxpi_ros/vmxpi_ros/srv/StringRes.srv" NAME_WE)
add_dependencies(vmxpi_ros_generate_messages_py _vmxpi_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vmxpi_ros_genpy)
add_dependencies(vmxpi_ros_genpy vmxpi_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vmxpi_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vmxpi_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(vmxpi_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(vmxpi_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(vmxpi_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_srvs_generate_messages_cpp)
  add_dependencies(vmxpi_ros_generate_messages_cpp std_srvs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vmxpi_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(vmxpi_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(vmxpi_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(vmxpi_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_srvs_generate_messages_eus)
  add_dependencies(vmxpi_ros_generate_messages_eus std_srvs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vmxpi_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(vmxpi_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(vmxpi_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(vmxpi_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_srvs_generate_messages_lisp)
  add_dependencies(vmxpi_ros_generate_messages_lisp std_srvs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vmxpi_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(vmxpi_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(vmxpi_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(vmxpi_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_srvs_generate_messages_nodejs)
  add_dependencies(vmxpi_ros_generate_messages_nodejs std_srvs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vmxpi_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(vmxpi_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(vmxpi_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(vmxpi_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_srvs_generate_messages_py)
  add_dependencies(vmxpi_ros_generate_messages_py std_srvs_generate_messages_py)
endif()
