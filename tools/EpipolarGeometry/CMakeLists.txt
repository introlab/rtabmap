
IF(MINGW)
  ADD_EXECUTABLE(epipolar_geometry WIN32 main.cpp)
ELSE()
  ADD_EXECUTABLE(epipolar_geometry main.cpp)
ENDIF()
TARGET_LINK_LIBRARIES(epipolar_geometry rtabmap_gui)

SET_TARGET_PROPERTIES( epipolar_geometry 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-epipolar_geometry)

