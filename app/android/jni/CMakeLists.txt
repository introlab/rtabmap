
SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/tango-gl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    "${ANDROID_NDK}/platforms/android-${ANDROID_NATIVE_API_LEVEL}/arch-${ANDROID_ARCH_NAME}/usr/include"
)

SET(LIBRARIES
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES}
)

set(sources
  jni_interface.cpp
  CameraMobile.cpp
  RTABMapApp.cpp
  scene.cpp
  point_cloud_drawable.cpp
  graph_drawable.cpp
  tango-gl/axis.cpp 
  tango-gl/camera.cpp 
  tango-gl/conversions.cpp 
  tango-gl/drawable_object.cpp 
  tango-gl/frustum.cpp 
  tango-gl/gesture_camera.cpp 
  tango-gl/grid.cpp 
  tango-gl/line.cpp 
  tango-gl/shaders.cpp 
  tango-gl/trace.cpp 
  tango-gl/transform.cpp 
  tango-gl/util.cpp
)

IF(Tango_FOUND)

  SET(sources
    ${sources}
    CameraTango.cpp
  )
  SET(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    ${Tango_INCLUDE_DIRS}
  )
  SET(LIBRARIES
    ${LIBRARIES}
    ${Tango_LIBRARIES}
  )
  
  file(COPY ${Tango_support_LIBRARY} 
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME})
   
ENDIF(Tango_FOUND)

IF(ARCore_FOUND)

  SET(sources
    ${sources}
    CameraARCore.cpp
  )
  SET(INCLUDE_DIRS
    ${INCLUDE_DIRS}
    ${ARCore_INCLUDE_DIRS}
  )
  SET(LIBRARIES
    ${LIBRARIES}
    ${ARCore_LIBRARIES}
  )
 
  file(COPY ${ARCore_LIBRARY} 
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME})
    
ENDIF(ARCore_FOUND)

add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

add_library(NativeRTABMap SHARED ${sources})        
target_link_libraries(NativeRTABMap ${LIBRARIES}
  android
  log
  GLESv2
  camera2ndk
  mediandk
  rtabmap_core
  rtabmap_utilite
)

# see ant.properties.in
set_target_properties(NativeRTABMap PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}"
         LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}"
         LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}")
