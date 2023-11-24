
SET(SRC_FILES
    main.cpp
)

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

IF(BUILD_AS_BUNDLE)
  ADD_DEFINITIONS("-DBUILD_AS_BUNDLE")
ENDIF()

# Add binary
IF(APPLE AND BUILD_AS_BUNDLE)
  ADD_EXECUTABLE(rtabmap_app MACOSX_BUNDLE ${SRC_FILES})
ELSEIF(WIN32 AND BUILD_AS_BUNDLE)
  ADD_EXECUTABLE(rtabmap_app WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(rtabmap_app ${SRC_FILES})
ENDIF()
TARGET_LINK_LIBRARIES(rtabmap_app rtabmap_gui)

IF(APPLE AND BUILD_AS_BUNDLE)
  SET_TARGET_PROPERTIES(rtabmap_app PROPERTIES
    OUTPUT_NAME ${CMAKE_BUNDLE_NAME})
ELSEIF(WIN32)
  SET_TARGET_PROPERTIES(rtabmap_app PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME})
ELSE()
  SET_TARGET_PROPERTIES(rtabmap_app PROPERTIES
    OUTPUT_NAME ${PROJECT_PREFIX})
ENDIF()

#---------------------------
# Installation stuff
#--------------------------- 
INSTALL(TARGETS rtabmap_app
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

  IF(Qt6_FOUND)
	# Reference: https://doc-snapshots.qt.io/qt6-6.4/qt-deploy-runtime-dependencies.html
	# The following script must only be executed at install time
	set(deploy_script "${CMAKE_CURRENT_BINARY_DIR}/deploy_app$<CONFIG>.cmake")
	file(GENERATE OUTPUT ${deploy_script} CONTENT "
	# Including the file pointed to by QT_DEPLOY_SUPPORT ensures the generated
	# deployment script has access to qt_deploy_runtime_dependencies()
	include(\"${QT_DEPLOY_SUPPORT}\")
	qt_deploy_runtime_dependencies(
		EXECUTABLE \"${APPS}\"
		VERBOSE
		PLUGINS_DIR ${plugin_dest_dir}/plugins
	)")
	install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appDebug.cmake"
		CONFIGURATIONS Debug
		COMPONENT runtime)
	install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appRelease.cmake"
		CONFIGURATIONS Release
		COMPONENT runtime)
	install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appRelWithDebInfo.cmake"
		CONFIGURATIONS RelWithDebInfo
		COMPONENT runtime)
	install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appMinSizeRel.cmake"
		CONFIGURATIONS MinSizeRel
		COMPONENT runtime)
  ELSEIF(Qt5_FOUND)
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
  ELSEIF(QT_PLUGINS_DIR) # Qt4
    # Install needed Qt plugins by copying directories from the qt installation
	# One can cull what gets copied by using 'REGEX "..." EXCLUDE'
	# Exclude debug libraries
	INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats"
		DESTINATION ${plugin_dest_dir}/plugins 
		COMPONENT runtime 
		REGEX ".*d4.dll" EXCLUDE
		REGEX ".*d4.a" EXCLUDE)
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
  SET(DIRS ${QT_LIBRARY_DIRS} ${PROJECT_BINARY_DIR}/bin)
  IF(APPLE)
    SET(DIRS ${DIRS} /usr/local /usr/local/lib /opt/homebrew /opt/homebrew/lib /opt/homebrew/lib/gcc/current)
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
  
