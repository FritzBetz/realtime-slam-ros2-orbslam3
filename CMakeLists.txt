cmake_minimum_required(VERSION 3.8)
project(orbslam_cpp)

set(USER_HOME $ENV{HOME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(ORB_SLAM3 REQUIRED)

add_executable(orbslam3_localization src/orbslam3_localization.cpp)


# Assumes ORB_SLAM3 is installed in /home/(youruser)/dev



target_link_libraries(orbslam3_localization
  ${OpenCV_LIBS}
  ${USER_HOME}/dev/ORB_SLAM3/lib/libORB_SLAM3.so
)
target_include_directories(orbslam3_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${USER_HOME}/dev/ORB_SLAM3/include
  ${USER_HOME}/dev/ORB_SLAM3
  ${USER_HOME}/dev/ORB_SLAM3/Thirdparty/Sophus
  ${USER_HOME}/dev/ORB_SLAM3/include/CameraModels
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

target_compile_features(orbslam3_localization PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  orbslam3_localization
  Pangolin
  geometry_msgs
  tf2
  tf2_ros
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

install(TARGETS orbslam3_localization
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
