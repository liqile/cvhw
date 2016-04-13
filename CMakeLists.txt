cmake_minimum_required(VERSION 2.8)
project(myslam)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters)
ADD_DEFINITIONS(${PCL_DEFINITIONS})
#find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/frame
${PROJECT_SOURCE_DIR}/include/image
${PROJECT_SOURCE_DIR}/include/loop
${PROJECT_SOURCE_DIR}/include/map
${PROJECT_SOURCE_DIR}/include/setting
${PROJECT_SOURCE_DIR}/include/track
${PROJECT_SOURCE_DIR}/include/optimizer
${PROJECT_SOURCE_DIR}/include/display
${PROJECT_SOURCE_DIR}/include/display/drawer
${PROJECT_SOURCE_DIR}/include/converter
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

LINK_LIBRARIES(${PCL_LIBRARY_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
#src/Tracking.cc
src/setting/parameter.cc
src/frame/Frame.cc
src/frame/Pose.cc
#src/frame/KeyFrame.cc
src/image/ORBextractor.cc
src/image/ORBmatcher.cc
src/System.cc
src/image/Features.cc
src/image/ORBVocabulary.cc
src/map/Map.cc
src/map/MapPoint.cc
src/optimizer/Optimizer.cc
src/track/Tracking.cc
src/display/display.cc
src/display/drawer/TrackingDrawer.cc
src/display/drawers.cc
src/converter/Converter.cc
#src/Converter.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PCL_LIBRARIES}
#${Pangolin_LIBRARIES}
)
add_executable(main
main.cc)
target_link_libraries(main ${PROJECT_NAME})

#add_executable(testmain
#main.cpp
#)
#target_link_libraries(testmain ${OpenCV_LIBS})
