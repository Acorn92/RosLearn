cmake_minimum_required(VERSION 3.0.2)
project(lsb5)
set(CMAKE_CXX_STANDARD 17)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

# указываем в проекте все дериктории с заголовочными файлами
include_directories(
  include

  ${catkin_INCLUDE_DIRS}
)


# добавляем библиотеки для сборки
# библиотека систем управления
add_library(my_control_sys_lib STATIC src/uav_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(my_uav_circle_fly_example src/uav_circle_fly_example.cpp)
add_executable(my_uav_controller_node src/uav_controller_node.cpp)

# libraris link for visual navigation system
target_link_libraries(my_control_sys_lib  ${catkin_LIBRARIES})
target_link_libraries(my_uav_circle_fly_example ${catkin_LIBRARIES})
target_link_libraries(my_uav_controller_node  ${catkin_LIBRARIES} my_control_sys_lib)

