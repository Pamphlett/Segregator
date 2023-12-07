# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lsegregator;-lnano_gicp;-lnanoflann".split(';') if "-lsegregator;-lnano_gicp;-lnanoflann" != "" else []
PROJECT_NAME = "segregator"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
