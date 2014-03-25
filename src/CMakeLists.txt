remake_ros_package(
  can_prius
  DEPENDS roscpp diagnostic_updater std_msgs
  EXTRA_BUILD_DEPENDS libcan-prius-dev
  EXTRA_RUN_DEPENDS libcan-prius
  DESCRIPTION "Driver for Toyota PRIUS CAN bus."
)
remake_ros_package_config_generate(PACKAGE can_prius EXTRA_CFLAGS -std=c++0x)
