cmake_minimum_required(VERSION 3.10)
project(xsfm-data-converter)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(exiv2 CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)

include_directories(undistortFunctions)

add_executable(main undistortFunctions/main.cpp undistortFunctions/ocam_functions.cpp)

target_link_libraries(main ${OpenCV_LIBS} tinyxml2::tinyxml2 Exiv2::exiv2lib glog::glog fmt::fmt)