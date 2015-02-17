remake_ros_package(
  velodyne_post
  DEPENDS roscpp rospy rosbash velodyne sensor_msgs
  EXTRA_BUILD_DEPENDS libvelodyne-dev libsnappy-dev
  EXTRA_RUN_DEPENDS libvelodyne libsnappy1
  DESCRIPTION "Post-processor for Velodyne HDL devices."
)
remake_ros_package_config_generate(PACKAGE velodyne_post
  EXTRA_CFLAGS -std=c++0x)
