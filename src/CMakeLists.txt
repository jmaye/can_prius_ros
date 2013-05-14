remake_ros_package(
  can_prius
  DEPENDS roscpp diagnostic_updater
  EXTRA_BUILD_DEPENDS libcan-prius-dev
  EXTRA_RUN_DEPENDS libcan-prius
  DESCRIPTION "Driver for Toyota PRIUS CAN bus."
)
