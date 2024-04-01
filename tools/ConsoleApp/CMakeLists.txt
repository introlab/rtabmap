
# Add binary called "consoleApp" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(consoleApp main.cpp)
TARGET_LINK_LIBRARIES(consoleApp rtabmap_core)

SET_TARGET_PROPERTIES( consoleApp 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-console)
  
INSTALL(TARGETS consoleApp
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)

