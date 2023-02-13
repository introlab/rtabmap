
ADD_EXECUTABLE(imagesJoiner main.cpp)
TARGET_LINK_LIBRARIES(imagesJoiner rtabmap_core)

SET_TARGET_PROPERTIES( imagesJoiner 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-imagesJoiner)

