#--------------------------------------------------------
# The CMakeLists.txt for: iArduRoverBridge
# Author(s): Jeremy Wenger
# Date: November 2025
#--------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(SRC
  iArduRoverBridge.cpp
  BoatState.cpp
  iArduRoverBridge_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iArduRoverBridge ${SRC})

TARGET_LINK_LIBRARIES(iArduRoverBridge
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   MAVSDK::mavsdk
   apputil
   mbutil
   m
   pthread
)
