#=======================================================================
# FILE:  moos-ivp-remora/CMakeLists.txt
# DATE:  2026/10/08
# INFO:  Top-level CMakeLists.txt for the Project Remora MOOS-IvP tree.
#        Layout follows moos-ivp-extend (pavlab-mit/moos-ivp-extend).
# NAME:  Jeremy Wenger
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
PROJECT( IVP_REMORA )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#=======================================================================
# Output directories for binaries and libraries
#=======================================================================
GET_FILENAME_COMPONENT(IVP_EXTEND_BIN_DIR "${CMAKE_SOURCE_DIR}/bin"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_LIB_DIR "${CMAKE_SOURCE_DIR}/lib"  ABSOLUTE )

SET( LIBRARY_OUTPUT_PATH      "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( ARCHIVE_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( LIBRARY_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )

SET( EXECUTABLE_OUTPUT_PATH    "${IVP_EXTEND_BIN_DIR}" CACHE PATH "" )
SET( RUNTIME_OUTPUT_DIRECTORY "${IVP_EXTEND_BIN_DIR}"  CACHE PATH "" )

#=======================================================================
# Find the moos-ivp source tree.
#
# Search order:
#   1. -DMOOSIVP_SOURCE_TREE_BASE=... on the cmake command line
#   2. $MOOS_IVP_ROOT in the environment (ansible/ sets this on the Jetson)
#   3. a sibling of this repo: ../../moos-ivp (laptop: ~/moos/moos-ivp
#      next to ~/moos/2.733)
#   4. ~/moos-ivp, ~/moos/moos-ivp
#=======================================================================
set(MOOSIVP_SEARCH_PATHS
    "$ENV{MOOS_IVP_ROOT}"
    "${CMAKE_SOURCE_DIR}/../moos-ivp"
    "${CMAKE_SOURCE_DIR}/../../moos-ivp"
    "${CMAKE_SOURCE_DIR}/../../../moos-ivp"
    "$ENV{HOME}/moos-ivp"
    "$ENV{HOME}/moos/moos-ivp"
)

find_path( MOOSIVP_SOURCE_TREE_BASE
           NAMES build-ivp.sh build-moos.sh
           PATHS ${MOOSIVP_SEARCH_PATHS}
           DOC "Base directory of the MOOS-IvP source tree"
           NO_DEFAULT_PATH
)

if (NOT MOOSIVP_SOURCE_TREE_BASE)
    message("Could not find the moos-ivp source tree. Looked in:")
    foreach(p ${MOOSIVP_SEARCH_PATHS})
        message("  ${p}")
    endforeach()
    message("Set MOOS_IVP_ROOT in the environment, or pass")
    message("  cmake -DMOOSIVP_SOURCE_TREE_BASE=/path/to/moos-ivp")
    message( FATAL_ERROR "CMake will exit." )
endif()
message(STATUS "moos-ivp tree: ${MOOSIVP_SOURCE_TREE_BASE}")

#=======================================================================
# Find MOOS and MOOSGeodesy (built inside the moos-ivp tree)
#=======================================================================
list(APPEND CMAKE_PREFIX_PATH
    "${MOOSIVP_SOURCE_TREE_BASE}/build/MOOS/MOOSCore"
    "${MOOSIVP_SOURCE_TREE_BASE}/build/MOOS/MOOSGeodesy")

find_package(MOOS 10.0 REQUIRED)
INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS})

find_package(MOOSGeodesy REQUIRED)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})
link_directories(${MOOSGeodesy_LIBRARY_PATH})

#=======================================================================
# IvP headers and libraries
#=======================================================================
FILE(GLOB IVP_INCLUDE_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/ivp/src/lib_* )
INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

FILE(GLOB IVP_LIBRARY_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/lib )
LINK_DIRECTORIES(${IVP_LIBRARY_DIRS})

#=======================================================================
# Compiler flags (Linux and macOS)
#=======================================================================
if (NOT WIN32)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -Wdeprecated-declarations")
   IF(CMAKE_COMPILER_IS_GNUCXX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
   ELSE()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
   ENDIF()
endif()

#=======================================================================
# Optional dependencies. iArduRoverBridge needs MAVSDK (v2 or v3 C++ API).
#   macOS:  brew install mavsdk
#   Jetson: ansible/ (role mavsdk) builds it from source
#=======================================================================
find_package(MAVSDK QUIET)
if(MAVSDK_FOUND)
    message(STATUS "MAVSDK found: iArduRoverBridge will be built")
else()
    message(WARNING "MAVSDK not found: iArduRoverBridge will NOT be built. "
                    "macOS: brew install mavsdk. Jetson: run the ansible playbook.")
endif()

#=======================================================================
# Add Subdirectories
#=======================================================================
ADD_SUBDIRECTORY( src )
