cmake_minimum_required(VERSION 2.8.11)
include(CheckSymbolExists)

#========================
# Project details / setup
#========================

# Extract version from header, done first to satisfy CMP0048,
# see `cmake --help-policy CMP0048` for more information.
file(
	STRINGS                         # Read in a file to memory.
	pleno/version.h           		# File to parse for version number.
	PLENO_PROJECT_VERSION          	# Where to store the results (should just be one line)
	REGEX "#define PLENO_VERSION " 	# The space after is important to ignore 'PLENO_VERSION_INT'
)
# If no matches found, something is wrong with PointMatcher.h
if (NOT PLENO_PROJECT_VERSION)
	message(SEND_ERROR "Cannot find version number in '${CMAKE_CURRENT_SOURCE_DIR}/pleno/version.h'.")
endif (NOT PLENO_PROJECT_VERSION)
# Transform '#define PLENO_VERSION "X.Y.Z"' into 'X.Y.Z'
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PLENO_PROJECT_VERSION "${PLENO_PROJECT_VERSION}")

# In 3.0+, project(...) should specify VERSION to satisfy CMP0048
if (CMAKE_VERSION VERSION_LESS 3.0.0)
	project(libpleno)
else (CMAKE_VERSION VERSION_LESS 3.0.0)
	cmake_policy(SET CMP0048 NEW)
	project(libpleno VERSION ${PLENO_PROJECT_VERSION})
endif (CMAKE_VERSION VERSION_LESS 3.0.0)

# Ensure proper build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)


#================= extra building definitions ==============================
if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

add_definitions(-fconcepts)

# For Windows
if( MSVC )
	add_definitions( /D _VARIADIC_MAX=10 ) # VS2012 does not support tuples correctly yet
	add_definitions( /D _USE_MATH_DEFINES) # defines M_PI for Visual Studio
	add_definitions( /D _ENABLE_EXTENDED_ALIGNED_STORAGE) # this variable must be defined with VS2017 to acknowledge alignment changes of aligned_storage
endif()

#======================= installation =====================================

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")

if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/cmake/pleno)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

#===========================================================================

#======================== External Dependencies ===============================

# initially
set(EXTERNAL_LIBS "")
set(EXTRA_DEPS "")

# compile local version of lib
#add_subdirectory(contrib)

#--------------------
# DEPENDENCY:  libv-core, libv-graphic, libv-geometry, libv-lma
#--------------------
find_package(libv REQUIRED core lma graphic geometry)
include_directories(${LIBV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${LIBV_LIBRARIES})

#--------------------
# DEPENDENCY: eigen 3
#--------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${EIGEN})

find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/include/eigen3
)
# Suppress Eigen's warning by adding it to the system's library
include_directories(SYSTEM "${EIGEN_INCLUDE_DIR}")

#--------------------
# DEPENDENCY: OpenCV
#--------------------
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OpenCV_LIBS})

#--------------------
# DEPENDENCY: OpenGL
#--------------------
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})
set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OPENGL_LIBRARIES})

#--------------------
# DEPENDENCY: OpenMP (optional)
#--------------------
set(USE_OPEN_MP FALSE CACHE BOOL "Set to TRUE to use OpenMP")
if (USE_OPEN_MP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		add_definitions(-fopenmp -DHAVE_OPENMP)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		if (CMAKE_COMPILER_IS_GNUCC)
			set(EXTERNAL_LIBS ${EXTERNAL_LIBS} gomp)
    	message("-- OpenMP found, parallel computer enabled")
		endif(CMAKE_COMPILER_IS_GNUCC)
	endif(OPENMP_FOUND)
endif (USE_OPEN_MP)
#============================= end dependencies =================================


#========================== libpleno itself ==============================

# Check the compiler version as we need full C++11 support.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	# using Clang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.3")
		message(WARNING, "Your clang compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 3.3 or later is supported")
	endif ()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	# using AppleClang
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(WARNING "Your XCode environment has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 5.1 or later is supported")
	endif()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	# using GCC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.8.2")
		message(WARNING, "Your g++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version 4.8.2 or later is supported")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	# using MSVC
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.0.23506")
		message(WARNING "Your Microsoft Visual C++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while only version MSVC 2015 Update 1+ or later is supported")
	endif()
endif ()

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.8")
	if (MSVC)
		message(FATAL_ERROR, "CMake version 3.8 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
	endif ()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set (CMAKE_CXX_FLAGS "-std=c++1z ${CMAKE_CXX_FLAGS}")
	else ()
		set (CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
	endif ()
else ()
	set (CMAKE_CXX_STANDARD 17)
endif ()

# SOURCE

# Pleno lib and install
set(PLENO_GEOMETRY_SRC
	pleno/geometry/transformation.cpp
	pleno/geometry/distortions.cpp
	pleno/geometry/mesh.cpp
	pleno/geometry/pose.cpp
	pleno/geometry/ray.cpp
	pleno/geometry/sensor.cpp
	pleno/geometry/observation.cpp
	pleno/geometry/internals.cpp
	pleno/geometry/mia.cpp
	pleno/geometry/mla.cpp
	pleno/geometry/plane.cpp
	pleno/geometry/rgba.cpp
#--Camera	
	pleno/geometry/camera/camera.cpp
	pleno/geometry/camera/unified.cpp
	pleno/geometry/camera/orthographic.cpp
	pleno/geometry/camera/pinhole.cpp
	pleno/geometry/camera/thinlens.cpp
	pleno/geometry/camera/plenoptic.cpp
#--Model
	pleno/geometry/model/microimage.cpp	
#--Depth	
	pleno/geometry/depth/depthmap.cpp
	pleno/geometry/depth/pointcloud.cpp
	pleno/geometry/depth/depthimage.cpp
)

set(PLENO_OPTIMIZATION_SRC
	pleno/optimization/pose2d.cpp
	pleno/optimization/pose3d.cpp
	pleno/optimization/grid3d.cpp
	pleno/optimization/mia.cpp
	pleno/optimization/mla.cpp
	pleno/optimization/focal.cpp
	pleno/optimization/distortions.cpp
	pleno/optimization/thinlens.cpp
	pleno/optimization/sensor.cpp
	pleno/optimization/homography.cpp
	pleno/optimization/kappa.cpp
	pleno/optimization/depth.cpp
	pleno/optimization/functions.cpp
#--Errors
	pleno/optimization/errors/grid.cpp
	pleno/optimization/errors/mic.cpp
	pleno/optimization/errors/bap.cpp
	pleno/optimization/errors/corner.cpp
	pleno/optimization/errors/radius.cpp
	pleno/optimization/errors/blur.cpp
	pleno/optimization/errors/extrinsics.cpp
	pleno/optimization/errors/invdistortion.cpp
	pleno/optimization/errors/disparity.cpp
	pleno/optimization/errors/scaling.cpp
	pleno/optimization/errors/lidarcamera.cpp
)

set(PLENO_IO_SRC
	pleno/io/printer.cpp
	pleno/io/images.cpp
	pleno/io/depths.cpp
)

set(PLENO_GRAPHIC_SRC
	pleno/graphic/viewer_3d.cpp
	pleno/graphic/viewer_2d.cpp
	pleno/graphic/gui.cpp
	pleno/graphic/draw.cpp
	pleno/graphic/colormap.cpp
)

set(PLENO_PROCESSING_SRC
	pleno/processing/estimation.cpp
#--Tools
	pleno/processing/tools/chrono.cpp	
#--Precalibration
	pleno/processing/precalibration/preprocess.cpp
	pleno/processing/precalibration/display.cpp
#--Calibration
	pleno/processing/calibration/mia.cpp	
	pleno/processing/calibration/plenoptic.cpp	
	pleno/processing/calibration/extrinsics.cpp	
	pleno/processing/calibration/blur.cpp	
	pleno/processing/calibration/invdistortion.cpp
	pleno/processing/calibration/scaling.cpp
	pleno/process