
find_package(Tango REQUIRED)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/tango-gl/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Tango_INCLUDE_DIRS}
    "${ANDROID_NDK}/platforms/android-${ANDROID_NATIVE_API_LEVEL}/arch-${ANDROID_ARCH_NAME}/usr/include"
)

SET(LIBRARIES
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES}
	${Tango_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

set(sources
  jni_interface.cpp
  CameraTango.cpp
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

add_library(NativeRTABMap SHARED ${sources})        
target_link_libraries(NativeRTABMap ${LIBRARIES}
  android
  log
  GLESv2
  rtabmap_core
  rtabmap_utilite
)

# see ant.properties.in
set_target_properties(NativeRTABMap PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}"
         LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}"
         LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/../libs/${ANDROID_NDK_ABI_NAME}")
