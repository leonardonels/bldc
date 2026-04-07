cmake_minimum_required(VERSION 3.8)
project(vesc_foc_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPS rclcpp std_msgs)
set(INC include)

add_executable(pmsm_plant src/pmsm_plant_node.cpp)
ament_target_dependencies(pmsm_plant ${DEPS})
target_include_directories(pmsm_plant PRIVATE ${INC})
target_compile_features(pmsm_plant PRIVATE cxx_std_17)

add_executable(vesc_controller src/vesc_controller_node.cpp)
ament_target_dependencies(vesc_controller ${DEPS})
target_include_directories(vesc_controller PRIVATE ${INC})
target_compile_features(vesc_controller PRIVATE cxx_std_17)

add_executable(ecu_sim src/ecu_sim_node.cpp)
ament_target_dependencies(ecu_sim ${DEPS})
target_compile_features(ecu_sim PRIVATE cxx_std_17)

install(TARGETS pmsm_plant vesc_controller ecu_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
