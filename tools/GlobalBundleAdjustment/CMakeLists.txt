
ADD_EXECUTABLE(globalBundleAdjustment main.cpp)
  
TARGET_LINK_LIBRARIES(globalBundleAdjustment rtabmap_core)

SET_TARGET_PROPERTIES( globalBundleAdjustment 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-globalBundleAdjustment)

INSTALL(TARGETS globalBundleAdjustment
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)


