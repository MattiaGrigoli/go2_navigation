cmake_minimum_required(VERSION 3.8)
project(go2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BodyHeight.srv"
  "srv/ContinuousGait.srv"
  "srv/Euler.srv"
  "srv/FootRaiseHeight.srv"
  "srv/GetBodyHeight.srv"
  "srv/GetBrightness.srv"
  "srv/GetFootRaiseHeight.srv"
  "srv/GetSpeedLevel.srv"
  "srv/GetState.srv"
  "srv/GetSwitch.srv"
  "srv/GetSwitchObstaclesAvoidance.srv"
  "srv/GetVolume.srv"
  "srv/Mode.srv"
  "srv/Pose.srv"
  "srv/Say.srv"
  "srv/SetBrightness.srv"
  "srv/SetSwitch.srv"
  "srv/SetSwitchObstaclesAvoidance.srv"
  "srv/SetVolume.srv"
  "srv/SpeedLevel.srv"
  "srv/SwitchGait.srv"
  "srv/SwitchJoystick.srv"
  DEPENDENCIES
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()