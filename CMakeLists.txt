cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(OpenMVG)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

#Check C++11 or C++0x support
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

find_package(Eigen3 QUIET)
find_package(JPEG QUIET)
find_package(PNG QUIET)
find_package(TIFF QUIET)
find_package(Boost REQUIRED COMPONENTS
            program_options
            filesystem
            graph
            system)
add_subdirectory(third_party)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party
  ${EIGEN3_INCLUDE_DIR}
)
include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/image_io.cpp
    src/numeric.cpp
    src/colmap_reader.cpp
)

#target_include_directories(${PROJECT_NAME}
#    ${PNG_INCLUDE_DIRS}
#    ${TIFF_INCLUDE_DIR}
#)

target_link_libraries(${PROJECT_NAME}
  ${EIGEN3_LIBS}
  ${TIFF_LIBRARIES}
  ${PNG_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${STLPLUS_LIBRARY}
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(openMVG_sample_pano_converter src/main_pano_converter.cpp)
target_link_libraries(openMVG_sample_pano_converter
  ${OPENMVG_LIBRARY_DEPENDENCIES}
  ${STLPLUS_LIBRARY}
  ${PROJECT_NAME}
  ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
