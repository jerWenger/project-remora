#--------------------------------------------------------
# The CMakeLists.txt for:                       iContactUDP
# Author(s):                              Jeremy Wenger
#--------------------------------------------------------

SET(SRC
  ContactUDP.cpp
  ContactUDP_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iContactUDP ${SRC})

TARGET_LINK_LIBRARIES(iContactUDP
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

