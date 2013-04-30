
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ../include
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "resource_tool" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(uresourcegenerator ${SRC_FILES})
TARGET_LINK_LIBRARIES(uresourcegenerator rtabmap_utilite)

SET_TARGET_PROPERTIES(
uresourcegenerator 
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
)