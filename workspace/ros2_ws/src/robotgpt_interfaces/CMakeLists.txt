cmake_minimum_required(VERSION 3.5)
project(robotgpt_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/StateReward.msg"
  "msg/State.msg"
  "msg/ObjectPose.msg"
  "msg/ObjectStatesRequest.msg"
  "msg/ObjectStates.msg"
  "msg/TrajCompletionMsg.msg"
  "msg/ResultEvaluation.msg"
  "msg/EECommandsM.msg"
  "msg/CodeExecutionM.msg"
  "msg/ResetRequest.msg"
  "msg/CodeError.msg"
  "msg/StartM.msg"
  "msg/StateEnv.msg"
  "msg/CropM.msg"
  "msg/CropRequest.msg"
  "srv/CodeExecution.srv"
  "srv/EECommands.srv"
  "srv/Trajectory.srv"
  "srv/EvaluationCode.srv"
  "srv/ObjectStatesR.srv"
  DEPENDENCIES sensor_msgs
)
ament_package()
