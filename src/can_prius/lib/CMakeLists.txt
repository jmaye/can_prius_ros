remake_find_package(libcan-prius CONFIG)
remake_include(${LIBCAN_PRIUS_INCLUDE_DIRS})

remake_ros_package_add_library(can_prius LINK ${LIBCAN_PRIUS_LIBRARIES})
