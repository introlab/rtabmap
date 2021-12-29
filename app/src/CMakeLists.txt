
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_BINARY_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${QT_LIBRARIES} 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# rc.exe has problems with these defintions... commented!
#add_definitions(${PCL_DEFINITIONS})

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# For Apple set the icns file containing icons
IF(APPLE AND BUILD_AS_BUNDLE)
  # set how it shows up in the Info.plist file
  SET(MACOSX_BUNDLE_ICON_FILE ${PROJECT_NAME}.icns) 
  # set where in the bundle to put the icns file
  SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  # include the icns file in the target
  SET(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.icns)
ENDIF(APPLE AND BUILD_AS_BUNDLE)

# Add exe icon resource
IF(WIN32)
  IF( MINGW )
    # resource compilation for MinGW
    ADD_CUSTOM_COMMAND( OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o
                        COMMAND windres.exe -I${CMAKE_CURRENT_SOURCE_DIR} -i${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.rc 
                             -o ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o )
    SET(SRC_FILES ${SRC_FILES} ${CMAKE_CURRENT_BINARY_DIR}/AppIco.o)
  ELSE( MINGW )
    SET(SRC_FILES ${SRC_FILES} ${PROJECT_NAME}.rc)
  ENDIF( MINGW )
ENDIF(WIN32)

# Add binary
IF(APPLE AND BUILD_AS_BUNDLE)
  ADD_EXECUTABLE(rtabmap MACOSX_BUNDLE ${SRC_FILES})
ELSEIF(MINGW)
  ADD_EXECUTABLE(rtabmap WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(rtabmap ${SRC_FILES})
ENDIF()
TARGET_LINK_LIBRARIES(rtabmap rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})
IF(Qt5_FOUND)
    IF(Qt5Svg_FOUND)
        QT5_USE_MODULES(rtabmap Widgets Core Gui Svg PrintSupport)
    ELSE()
        QT5_USE_MODULES(rtabmap Widgets Core Gui PrintSupport)
    ENDIF()
ENDIF(Qt5_FOUND)

IF(APPLE AND BUILD_AS_BUNDLE)
  SET_TARGET_PROPERTIES(rtabmap PROPERTIES
    OUTPUT_NAME ${CMAKE_BUNDLE_NAME})
ELSEIF(WIN32)
  SET_TARGET_PROPERTIES(rtabmap PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME})
ELSE()
  SET_TARGET_PROPERTIES(rtabmap PROPERTIES
    OUTPUT_NAME ${PROJECT_PREFIX})
ENDIF()

#---------------------------
# Installation stuff
#--------------------------- 
INSTALL(TARGETS rtabmap
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
      
IF(APPLE AND BUILD_AS_BUNDLE)
INSTALL(CODE "execute_process(COMMAND ln -s \"../MacOS/${CMAKE_BUNDLE_NAME}\" ${PROJECT_NAME}
        WORKING_DIRECTORY \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin)")
ENDIF(APPLE AND BUILD_AS_BUNDLE)
       
IF(BUILD_AS_BUNDLE AND (APPLE OR WIN32))
  SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}")
  SET(plugin_dest_dir bin)
  SET(qtconf_dest_dir bin)
  SET(openni2_dest_dir bin)
 
  IF(APPLE)
    SET(plugin_dest_dir MacOS)
    SET(qtconf_dest_dir Resources)
    SET(openni2_dest_dir MacOS)
    SET(APPS "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/MacOS/${CMAKE_BUNDLE_NAME}")
  ENDIF(APPLE)

  IF(OpenNI2_FOUND)
    # Install needed OpenNI sensor dll by copying directories from the OpenNI2 installation
    IF(WIN32)
      find_path(OpenNI2_BIN_DIR NAMES OpenNI2.dll)
      IF(NOT OpenNI2_BIN_DIR)
        MESSAGE(FATAL_ERROR "OpenNI2.dll not found! Verify your PATH.")
      ENDIF(NOT OpenNI2_BIN_DIR)
    ELSE()
      # Mac OS X
      get_filename_component(OpenNI2_BIN_DIR "${OpenNI2_LIBRARY}" PATH)
    ENDIF()
    
    INSTALL(DIRECTORY "${OpenNI2_BIN_DIR}/OpenNI2" 
          DESTINATION ${openni2_dest_dir} 
          COMPONENT runtime 
          REGEX ".*pdb" EXCLUDE)
    INSTALL(FILES "${OpenNI2_BIN_DIR}/OpenNI.ini" 
          DESTINATION ${openni2_dest_dir} 
          COMPONENT runtime)
  ENDIF(OpenNI2_FOUND)
  
  IF(k4a_FOUND)
	# Install needed depthengine_2_0.dll
    IF(WIN32)
		file(TO_CMAKE_PATH "$ENV{K4A_ROOT_DIR}" ENV_K4A_ROOT_DIR)
		INSTALL(FILES "${ENV_K4A_ROOT_DIR}/tools/depthengine_2_0.dll" 
          DESTINATION ${plugin_dest_dir} 
          COMPONENT runtime)
	ENDIF(WIN32)
  ENDIF(k4a_FOUND)
  
  IF(Torch_FOUND)
	# Install needed cudnn_ops_infer64_8.dll and cudnn_cnn_infer64_8.dll
	# TODO: should be a more general way to include them if version is different
    IF(WIN32 AND CUDA_FOUND)
		find_file(CUDNN_OPS_DLL NAMES cudnn_ops_infer64_8.dll)
		find_file(CUDNN_CNN_DLL NAMES cudnn_cnn_infer64_8.dll)
		IF(CUDNN_OPS_DLL AND CUDNN_CNN_DLL)
			MESSAGE(STATUS "Found ${CUDNN_OPS_DLL}")
			MESSAGE(STATUS "Found ${CUDNN_CNN_DLL}")
			INSTALL(FILES ${CUDNN_OPS_DLL} ${CUDNN_CNN_DLL}
			  DESTINATION ${plugin_dest_dir} 
			  COMPONENT runtime)
		ELSE()
			MESSAGE(AUTHOR_WARNING "Using Torch with CUDA, but cudnn_ops_infer64_8.dll and cudnn_cnn_infer64_8.dll are not found on the PATH, so it won't be added to package.")
		ENDIF()
	ENDIF(WIN32 AND CUDA_FOUND)
  ENDIF(Torch_FOUND)

  # Install needed Qt plugins by copying directories from the qt installation
  # One can cull what gets copied by using 'REGEX "..." EXCLUDE'
  # Exclude debug libraries
  IF(QT_PLUGINS_DIR)
    INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats"
          DESTINATION ${plugin_dest_dir}/plugins 
          COMPONENT runtime 
          REGEX ".*d4.dll" EXCLUDE
          REGEX ".*d4.a" EXCLUDE)
  ELSE()
	#Qt5
	foreach(plugin ${Qt5Gui_PLUGINS})
		get_target_property(plugin_loc ${plugin} LOCATION)
		get_filename_component(plugin_dir ${plugin_loc} DIRECTORY)
		string(REPLACE "plugins" ";" loc_list ${plugin_dir})
		list(GET loc_list 1 plugin_type)
		IF(NOT plugin_root)
			get_filename_component(plugin_root ${plugin_dir} DIRECTORY)
		ENDIF(NOT plugin_root)
		#MESSAGE(STATUS "Qt5 plugin \"${plugin_loc}\" installed in \"${plugin_dest_dir}/plugins${plugin_type}\"")
		INSTALL(FILES ${plugin_loc}
          DESTINATION ${plugin_dest_dir}/plugins${plugin_type}
          COMPONENT runtime)
	endforeach()
	IF(NOT Qt5Widgets_VERSION VERSION_LESS 5.10.0)
		IF(WIN32)
			SET(plugin_loc "${plugin_root}/styles/qwindowsvistastyle.dll")
		ELSEIF(APPLE)
			SET(plugin_loc "${plugin_root}/styles/libqmacstyle.dylib")
		ENDIF()
		IF(EXISTS ${plugin_loc})
			get_filename_component(plugin_dir ${plugin_loc} DIRECTORY)
			string(REPLACE "plugins" ";" loc_list ${plugin_dir})
			list(GET loc_list 1 plugin_type)
			INSTALL(FILES ${plugin_loc}
				DESTINATION ${plugin_dest_dir}/plugins${plugin_type}
				COMPONENT runtime)
			#MESSAGE(STATUS "Qt5 plugin \"${plugin_loc}\" installed in \"${plugin_dest_dir}/plugins${plugin_type}\"")
		ENDIF(EXISTS ${plugin_loc})
	ENDIF(NOT Qt5Widgets_VERSION VERSION_LESS 5.10.0)
  ENDIF()
  
  # install a qt.conf file
  # this inserts some cmake code into the install script to write the file
  SET(QT_CONF_FILE [Paths]\nPlugins=plugins)
  IF(APPLE)
    SET(QT_CONF_FILE [Paths]\nPlugins=MacOS/plugins)
  ENDIF(APPLE)
  INSTALL(CODE "
    file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"${QT_CONF_FILE}\")
    " COMPONENT runtime)

  # directories to look for dependencies
  SET(DIRS ${QT_LIBRARY_DIRS} ${PROJECT_SOURCE_DIR}/bin)
  IF(APPLE)
    SET(DIRS ${DIRS} /usr/local /usr/local/lib)
  ENDIF(APPLE)

  # Now the work of copying dependencies into the bundle/package
  # The quotes are escaped and variables to use at install time have their $ escaped
  # An alternative is the do a configure_file() on a script and use install(SCRIPT  ...).
  # Note that the image plugins depend on QtSvg and QtXml, and it got those copied
  # over.
  # To find dependencies, cmake use "otool" on Apple and "dumpbin" on Windows (make sure you have one of them).
  install(CODE "
   file(GLOB_RECURSE QTPLUGINS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    set(BU_CHMOD_BUNDLE_ITEMS ON)
    include(\"BundleUtilities\")
    fixup_bundle(\"${APPS}\" \"\${QTPLUGINS}\" \"${DIRS}\")
  " COMPONENT runtime)
ENDIF(BUILD_AS_BUNDLE AND (APPLE OR WIN32))
  
