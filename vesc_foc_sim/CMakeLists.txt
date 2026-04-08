cmake_minimum_required(VERSION 3.8)
project(vesc_foc_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DEPS rclcpp std_msgs)
set(BLDC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ── Firmware C library ────────────────────────────────────────────────────────
# Compiles the real VESC firmware sources as C (not C++) plus the bridge that
# wraps the complex C types with a plain scalar interface for C++ callers.
# -w suppresses upstream warnings we don't own.
add_library(vesc_firmware_c STATIC
    ${BLDC_ROOT}/motor/foc_math.c
    ${BLDC_ROOT}/util/utils_math.c
    src/foc_bridge.c
)
set_target_properties(vesc_firmware_c PROPERTIES LINKER_LANGUAGE C)
target_compile_options(vesc_firmware_c PRIVATE -w)
target_include_directories(vesc_firmware_c PRIVATE
    ${BLDC_ROOT}/tests/utils_math                       # stub ch.h — upstream test infrastructure, must come before any ChibiOS path
    ${BLDC_ROOT}                                        # datatypes.h
    ${BLDC_ROOT}/motor                                  # foc_math.h
    ${BLDC_ROOT}/util                                   # utils_math.h
)

# ── Sim executables ───────────────────────────────────────────────────────────
add_executable(pmsm_plant src/pmsm_plant_node.cpp)
ament_target_dependencies(pmsm_plant ${DEPS})
target_include_directories(pmsm_plant PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(pmsm_plant PRIVATE cxx_std_17)
target_compile_options(pmsm_plant PRIVATE -Wall -Wextra -O2)
target_link_libraries(pmsm_plant vesc_firmware_c)

add_executable(vesc_controller src/vesc_controller_node.cpp)
ament_target_dependencies(vesc_controller ${DEPS})
target_include_directories(vesc_controller PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(vesc_controller PRIVATE cxx_std_17)
target_compile_options(vesc_controller PRIVATE -Wall -Wextra -O2)
target_link_libraries(vesc_controller vesc_firmware_c)

add_executable(ecu_sim src/ecu_sim_node.cpp)
ament_target_dependencies(ecu_sim ${DEPS})
target_compile_features(ecu_sim PRIVATE cxx_std_17)
target_compile_options(ecu_sim PRIVATE -Wall -Wextra -O2)

install(TARGETS pmsm_plant vesc_controller ecu_sim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
