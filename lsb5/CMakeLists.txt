cmake_minimum_required(VERSION 3.0.2)
project(lsb5)
set(CMAKE_CXX_STANDARD 17)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  dynamic_reconfigure
  tf2
)
#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/uav_controller.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

# указываем в проекте все дериктории с заголовочными файлами
include_directories(
  include

  ${catkin_INCLUDE_DIRS}
)


# добавляем библиотеки для сборки
# библиотека систем управления
add_library(my_control_sys_lib STATIC src/uav_controller.cpp)
add_library(rotate_lib STATIC src/rotationTransform.cpp)

# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(my_uav_circle_fly_example src/uav_circle_fly_example.cpp)
add_executable(my_uav_controller_node src/uav_controller_node.cpp)
add_executable(my_go_to_example src/simple_flight_example.cpp)


add_dependencies(my_uav_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(my_control_sys_lib ${${PROJECT_NAME}_EXPORTED_TARGETS})

# libraris link for visual navigation system
target_link_libraries(my_control_sys_lib  ${catkin_LIBRARIES})
target_link_libraries(my_uav_circle_fly_example ${catkin_LIBRARIES})
target_link_libraries(my_uav_controller_node  ${catkin_LIBRARIES} my_control_sys_lib rotate_lib)
target_link_libraries(my_go_to_example  ${catkin_LIBRARIES})

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
