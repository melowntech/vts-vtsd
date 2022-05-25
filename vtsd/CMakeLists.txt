# bootstrap build system
cmake_minimum_required(VERSION 3.3)
project(vts-vtsd)
# by default, this is OSS build (can be overrided by customer machinery)
set(BUILDSYS_DEFAULT_CUSTOMER_NAME oss)
set(BUILDSYS_CPP_STANDARD 14)
include(buildsys/cmake/buildsys.cmake)

find_package(Boost 1.46 REQUIRED
  COMPONENTS thread program_options filesystem system date_time
             serialization regex chrono iostreams)
link_directories(${Boost_LIBRARY_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

if(${Boost_VERSION_MAJOR}.${Boost_VERSION_MINOR} VERSION_GREATER_EQUAL 1.73)
  # Since Boost.Python and Boost.Iostreams in Boost >=1.73 still include
  # deprecated headers we need to silence compiler
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS
    -DBOOST_ALLOW_DEPRECATED_HEADERS)
endif()

find_package(OpenCV REQUIRED)

find_package(GDAL REQUIRED)
include_directories(SYSTEM ${GDAL_INCLUDE_DIR})
find_package(Proj REQUIRED)
include_directories(${PROJ_INCLUDE_DIR})
find_package(GeographicLib REQUIRED)
include_directories(${GEOGRAPHICLIB_INCLUDE_DIR})

find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(magic REQUIRED)
find_package(CURL REQUIRED)
find_package(JsonCPP REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# dependencies
add_subdirectory(src/dbglog)
add_subdirectory(src/utility)
add_subdirectory(src/jsoncpp)
add_subdirectory(src/math)
add_subdirectory(src/geometry)
add_subdirectory(src/imgproc)
add_subdirectory(src/geo)
add_subdirectory(src/service)
add_subdirectory(src/half)
add_subdirectory(src/http)

add_subdirectory(src/roarchive)
add_subdirectory(src/slpk)

# vts libraries
if(DEFINED BUILDSYS_NOBUILD_TARGET_vts-tools)
  #  ???
  #  buildsys_nobuild_target(vts-libs)
endif()

add_subdirectory(src/vts-libs)

# NB: these must come after vts-libs for mesh support
add_subdirectory(src/gltf)
add_subdirectory(src/3dtiles)

if(NOT DEFINED BUILDSYS_NOBUILD_TARGET_vts-tools)
  # force build of vts tool
  add_custom_target(.vts ALL DEPENDS vts vts02vts vts2vts vts0)
endif()

# project
add_subdirectory(src/vtsd)
