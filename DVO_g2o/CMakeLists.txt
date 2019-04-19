cmake_minimum_required(VERSION 2.8.3)
project(DVO_g2o)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

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

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
if(Boost_FOUND)
  find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
  set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
  message("Boost not found!")
endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/g2o)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/g2o)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/Sophus
  ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/g2o
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(helper
  src/dvo_class.cpp 
  src/image_alignment.cpp 
  src/util.cpp
  src/Optimizer.cpp
  src/Converter.cc
  )

target_link_libraries(helper
   g2o
   ${EIGEN3_LIBS}
   ${OpenCV_LIBRARIES}
   ${Boost_GENERAL}
   ${Boost_LIBRARIES}
   gtsam
   )

add_executable(test_node src/main.cpp)
target_link_libraries(test_node
  helper
)
