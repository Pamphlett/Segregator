cmake_minimum_required(VERSION 3.10)
project(segregator)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        sensor_msgs
        nav_msgs
        pcl_ros
        cv_bridge
        eigen_conversions
)

find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(OpenMP)
find_package(PCL 1.8 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(Eigen3 3.2 QUIET REQUIRED NO_MODULE)

add_message_files(
        FILES
        cloud_info.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
)

catkin_package(
        INCLUDE_DIRS
          include
        LIBRARIES
          ${PROJECT_NAME}
          nano_gicp
          nanoflann
        CATKIN_DEPENDS 
          roscpp 
          rospy
          std_msgs
)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()


configure_file(cmake/pmc.CMakeLists.txt.in pmc-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
add_subdirectory("${CMAKE_BINARY_DIR}/pmc-src"   #CMAKE_BINARY_DIR =>
        "${CMAKE_BINARY_DIR}/pmc-build")

# spectra
configure_file(cmake/spectra.CMakeLists.txt.in spectra-download/CMakeLists.txt)
execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/spectra-download")
execute_process(COMMAND "${CMAKE_COMMAND}" --build .
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/spectra-download")
add_subdirectory("${CMAKE_BINARY_DIR}/spectra-src" # CMAKE_BINARY_DIR =>
        "${CMAKE_BINARY_DIR}/spectra-build")
set(SPECTRA_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/spectra-src/include")

include_directories(${SPECTRA_INCLUDE_DIRS})
# install(DIRECTORY ${SPECTRA_INCLUDE_DIRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

####### Executable ####### 
set(teaser_src
        src/registration.cc
        src/certification.cc
        src/graph.cc
        src/teaser_utils/feature_matcher.cc
        src/teaser_utils/fpfh.cc
        src/semantic_teaser.cpp
        )

add_executable(segregator_example examples/segregator_example.cpp ${teaser_src})
add_dependencies(segregator_example ${catkin_EXPORTED_TARGETS})
target_compile_options(segregator_example PRIVATE ${OpenMP_FLAGS})
target_link_libraries(segregator_example
        PUBLIC
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenMP_LIBS}
        stdc++fs
        pmc
        )

