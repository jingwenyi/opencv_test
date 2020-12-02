cmake_minimum_required( VERSION 2.8 )
project(Image_Mosaic_Demo)

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(Image_Mosaic_Demo  Image_Mosaic_Demo.cpp )
# 链接OpenCV库
target_link_libraries( Image_Mosaic_Demo  ${OpenCV_LIBS} )
