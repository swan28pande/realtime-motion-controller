cmake_minimum_required(VERSION 3.16)
project(realtime_motion_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(Threads REQUIRED)

add_library(motion STATIC src/can_bus.cpp src/servo_sim.cpp src/controller.cpp src/rt_thread.cpp src/operator_server.cpp)
target_include_directories(motion PUBLIC src)
target_link_libraries(motion PUBLIC Threads::Threads)

add_executable(motion_controller src/main.cpp)
target_link_libraries(motion_controller motion)

add_executable(servo_sim src/servo_sim_main.cpp)
target_link_libraries(servo_sim motion)
