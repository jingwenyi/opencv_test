cmake_minimum_required( VERSION 2.8 )
project(Image_project )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable(image_project  image_project.cpp )
# 链接OpenCV库
#target_link_libraries( image_project  ${OpenCV_LIBS} )


#add_executable(image_project2  image_project2.cpp)
#target_link_libraries(image_project2 ${OpenCV_LIBS})



add_executable(image_project3  image_project3.cpp)
target_link_libraries(image_project3 ${OpenCV_LIBS})

