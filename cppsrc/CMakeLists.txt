CMAKE_POLICY(SET CMP0048 NEW)
PROJECT (cppopt VERSION 1.0) #TODO name and version?
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

option(LIBONLY "Activate to build library only" ON)


########################################################################

# set paths ...
set(CMAKE_VERBOSE_MAKEFILE on)
SET(MYCODE_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/")
SET(EXTERNAL_DIR "${MYCODE_BASE_DIR}external/")

SET(OUTPUT_DIR "${MYCODE_BASE_DIR}build/")
#SET(MATLAB_BASE_DIR "/usr/local/MATLAB/R2020a/")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}bin")
SET(EXECUTABLE_OUTPUT_PATH  "${OUTPUT_DIR}bin")
SET(LIBRARY_OUTPUT_PATH "${OUTPUT_DIR}lib")

SET(MCMODEL_FLAG -mcmodel=large)


# find and add dependencies:
# include Eigen library:
SET(EIGEN_DIR "${EXTERNAL_DIR}eigen-3.4.0")
include_directories(${EIGEN_DIR})


# include boost libraries:
find_package( Boost 1.46.0 COMPONENTS serialization filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})


#find_package(Matlab)
SET(CMAKE_MODULE_PATH ${MYCODE_BASE_DIR}cmake)


message("Directories: ")
message("  MYCODE_BASE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message("  EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")
message("  DEPENDENCY_DIR = ${DEPENDENCY_DIR}")
message("  OUTPUT_DIR = ${OUTPUT_DIR}")
message("  CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")
message("  CMAKE_RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message("  EXECUTABLE_OUTPUT_PATH = ${EXECUTABLE_OUTPUT_PATH}")
message("  LIBRARY_OUTPUT_PATH = ${LIBRARY_OUTPUT_PATH}")
message("  CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")
message("  CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")


########################################################################

IF (CMAKE_COMPILER_IS_GNUCC)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++17 -fopenmp -Wno-sign-compare ")
	SET(CMAKE_CXX_RELEASE_FLAGS "${CMAKE_CXX_RELEASE_FLAGS} -O3 -msse2 -std=c++17  -std=gnu++0x -fopenmp -Wno-sign-compare ")
	##use c++0x flags for gcc (needed for the new integrators)
	#add_definitions(" -std=c++0x ")
	#add_definitions(" -std=gnu++0x ")
	set(COMPILE_FLAGS "${COMPILE_FLAGS} -fPIC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
ELSEIF(CMAKE_COMPILER_IS_GNUCC)
	message("Unsupported compiler. Flags are not set!")
ENDIF(CMAKE_COMPILER_IS_GNUCC)


MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()


########################################################################

message("processing util")
add_subdirectory(util)

message("processing math")
add_subdirectory(math)

message("processing models")
add_subdirectory(models)

message("processing functionReimplement")
add_subdirectory(functionReimplement)

#message("creating mex files")
#add_subdirectory(matlab_interface)
