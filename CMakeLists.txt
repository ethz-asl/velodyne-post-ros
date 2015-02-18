cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  ros-${ROS_DISTRIBUTION}-velodyne-post
  VERSION 0.1
  RELEASE alpha
  SUMMARY "ROS post-processor for Velodyne HDL devices."
  AUTHOR "Ralf Kaestner"
  AUTHOR "Jerome Maye"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/ethz-asl/velodyne-post-ros"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX velodyne-post
  INSTALL /usr
  README README.md
  EXTRA_CXX_FLAGS "-std=c++0x -Wall -Wextra"
)

remake_ros_pack_deb(DEFAULT velodyne_post)

remake_ros_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:kralf/asl
  PASS CMAKE_BUILD_TYPE ROS-${ROS_DISTRIBUTION}_VELODYNE_POST_GIT_REVISION
  FORCE_CONSISTENCY
  IF ${ROS_DISTRIBUTION} STREQUAL fuerte
)
remake_ros_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:kralf/asl
  PASS CMAKE_BUILD_TYPE ROS-${ROS_DISTRIBUTION}_VELODYNE_POST_GIT_REVISION
  FORCE_CONSISTENCY
  IF ${ROS_DISTRIBUTION} STREQUAL fuerte OR ${ROS_DISTRIBUTION}
    STREQUAL groovy
)
remake_ros_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  DEPENDS remake pkg-config
  UPLOAD ppa:kralf/asl
  PASS CMAKE_BUILD_TYPE ROS-${ROS_DISTRIBUTION}_VELODYNE_POST_GIT_REVISION
  FORCE_CONSISTENCY
  IF ${ROS_DISTRIBUTION} STREQUAL indigo
)
