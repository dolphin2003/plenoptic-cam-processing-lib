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
find_package(Ope