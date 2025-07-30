cmake_minimum_required(VERSION 3.31)
project(ArduinoRoboticArm)

set(CMAKE_CXX_STANDARD 20)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include_directories(${SDL2_INCLUDE_DIRS})

add_executable(ArduinoRoboticArm main.cpp)


target_link_libraries(ArduinoRoboticArm ${SDL2_LIBRARIES})