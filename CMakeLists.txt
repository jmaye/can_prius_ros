cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  can-prius-ros
  VERSION 0.1
  RELEASE alpha
  SUMMARY "ROS driver for Toyota PRIUS CAN bus."
  AUTHOR "Jerome Maye"
  CONTACT "jerome.maye@mavt.ethz.ch"
  HOME "http://github.com/jmaye/can_prius_ros"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX can-prius-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/can-prius
  EXTRA_CXX_FLAGS -std=c++0x
)

remake_ros_pack_deb()
