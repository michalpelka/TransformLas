cmake_minimum_required(VERSION 3.17)
project(test_las)

set(CMAKE_CXX_STANDARD 14)
add_subdirectory(3rd/LASzip/)

include_directories(3rd/LASzip/include)
find_package (Eigen3 3.3 NO_MODULE)

add_executable(transform transform.cpp)
target_link_libraries(transform laszip Eigen3::Eigen)