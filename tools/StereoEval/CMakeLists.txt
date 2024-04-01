
ADD_EXECUTABLE(stereoEval main.cpp)
TARGET_LINK_LIBRARIES(stereoEval rtabmap_core)

SET_TARGET_PROPERTIES( stereoEval 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-stereoEval)
 