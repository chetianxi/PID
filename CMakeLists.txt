cmake_minimum_required(VERSION 3.24)
project(PID_test)

set(CMAKE_CXX_STANDARD 17)

add_executable(PID_test main.cpp PID/PID.cpp PID/PID.h)
