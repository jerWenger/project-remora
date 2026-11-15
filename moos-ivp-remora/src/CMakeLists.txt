##############################################################################
# FILE:        moos-ivp-remora/src/CMakeLists.txt
# DESCRIPTION: Lists the apps, behaviors and libraries in this tree.
#              New app:      <moos-ivp>/scripts/GenMOOSApp_AppCasting pFoo p
#              New behavior: <moos-ivp>/scripts/GenBehavior Foo
#              then add the directory below.
##############################################################################

#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
if(MAVSDK_FOUND)
  ADD_SUBDIRECTORY(iArduRoverBridge)
endif()

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
