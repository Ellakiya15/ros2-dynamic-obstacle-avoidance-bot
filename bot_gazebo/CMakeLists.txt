cmake_minimum_required(VERSION 3.8)
project(bot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(
  DIRECTORY
    launch
    models
    worlds
    media
    plugins
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  PROGRAMS
    launch/spawn_action.py
  DESTINATION share/${PROJECT_NAME}
)

add_library(MoveCylinderPlugin SHARED src/MoveCylinderPlugin.cpp)
target_link_libraries(MoveCylinderPlugin ${GAZEBO_LIBRARIES})
target_include_directories(MoveCylinderPlugin PUBLIC ${GAZEBO_INCLUDE_DIRS})

ament_package()
