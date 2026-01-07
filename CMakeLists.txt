cmake_minimum_required(VERSION 3.31)
project(ArduinoRoboticArm)

set(CMAKE_CXX_STANDARD 20)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include_directories(${SDL2_INCLUDE_DIRS} include)

add_executable(ArduinoRoboticArm src/main.cpp
        src/Arm.cpp
        src/IKSolver.cpp
        src/Constants.cpp)


target_link_libraries(ArduinoRoboticArm ${SDL2_LIBRARIES})