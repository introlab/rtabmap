
ADD_EXECUTABLE(vocabularyComparison main.cpp)
TARGET_LINK_LIBRARIES(vocabularyComparison rtabmap_core)

SET_TARGET_PROPERTIES( vocabularyComparison 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-vocabularyComparison)

