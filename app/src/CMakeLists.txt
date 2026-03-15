
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
  ADD_EXECUTABLE(rtabmap_app ${SRC_FILES})
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
  SET(plugin_dest_dir bin/plugins)
  SET(qtconf_dest_dir bin)
  SET(thirdparty_dest_dir bin)
 
  IF(APPLE)
    SET(plugin_dest_dir MacOS/plugins)
    IF(Qt6_FOUND)
       SET(plugin_dest_dir PlugIns)
    ENDIF()
    SET(qtconf_dest_dir Resources)
    SET(thirdparty_dest_dir MacOS)
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
          DESTINATION ${thirdparty_dest_dir} 
          COMPONENT runtime 
          REGEX ".*pdb" EXCLUDE)
	IF(NOT WIN32)
		INSTALL(FILES "${OpenNI2_BIN_DIR}/OpenNI.ini" 
          DESTINATION ${thirdparty_dest_dir} 
          COMPONENT runtime)
	ENDIF()
  ENDIF(OpenNI2_FOUND)
  
  IF(k4a_FOUND)
	# Install needed depthengine_2_0.dll
    IF(WIN32)
		file(TO_CMAKE_PATH "$ENV{K4A_ROOT_DIR}" ENV_K4A_ROOT_DIR)
		INSTALL(FILES "${ENV_K4A_ROOT_DIR}/tools/depthengine_2_0.dll" 
          DESTINATION ${thirdparty_dest_dir} 
          COMPONENT runtime)
	ENDIF(WIN32)
  ENDIF(k4a_FOUND)
  
  IF(ZED_FOUND)
	# Install needed zlibwapi.dll
    IF(WIN32)
		file(TO_CMAKE_PATH "$ENV{ZED_SDK_ROOT_DIR}" ENV_ZED_SDK_ROOT_DIR)
		INSTALL(FILES "${ENV_ZED_SDK_ROOT_DIR}/bin/zlibwapi.dll" 
          DESTINATION ${thirdparty_dest_dir} 
          COMPONENT runtime)
	ENDIF(WIN32)
  ENDIF(ZED_FOUND)

  IF(OrbbecSDK_FOUND)
	# Install needed "extensions" folder
    IF(WIN32)
        find_path(OrbbecSDK_BIN_DIR NAMES OrbbecSDK.dll)
        IF(NOT OrbbecSDK_BIN_DIR)
            MESSAGE(FATAL_ERROR "OrbbecSDK.dll not found! Verify your PATH.")
        ENDIF(NOT OrbbecSDK_BIN_DIR)
		MESSAGE(FATAL "OrbbecSDK_BIN_DIR=${OrbbecSDK_BIN_DIR}")
        INSTALL(DIRECTORY "${OrbbecSDK_BIN_DIR}/extensions" 
          DESTINATION ${thirdparty_dest_dir} 
          COMPONENT runtime 
          FILES_MATCHING
          PATTERN "*.lib" EXCLUDE
          PATTERN "*")
	ENDIF(WIN32)
  ENDIF(OrbbecSDK_FOUND)
  
  IF(Torch_FOUND)
	# Install needed cudnn dlls
    IF(WIN32 AND CUDA_FOUND)
	    find_path(cuDNN_BIN_DIR NAMES cudnn.dll cudnn64.dll cudnn64_9.dll)
        IF(NOT cuDNN_BIN_DIR)
            MESSAGE(FATAL_ERROR "cudnn dlls not found! Make sure the dlls are in a directory on your PATH.")
        ENDIF(NOT cuDNN_BIN_DIR)
		MESSAGE(STATUS "cuDNN_BIN_DIR = ${cuDNN_BIN_DIR}")
		file(GLOB CUDNN_DLLS "${cuDNN_BIN_DIR}/cudnn*.dll")
		IF(CUDNN_DLLS)
			MESSAGE(STATUS "Found cuDNN DLLs: ${CUDNN_DLLS}")
			INSTALL(FILES ${CUDNN_DLLS}
					DESTINATION ${thirdparty_dest_dir} 
					COMPONENT runtime)
		ENDIF()
	ENDIF(WIN32 AND CUDA_FOUND)
  ENDIF(Torch_FOUND)
  
  set(python_pyd_dir "")
  IF(Python3_FOUND)
	# bundle python3
    IF(WIN32)
		set(python_pyd_dir "bin/Lib/site-packages")
		set(PYTHON_ZIP_NAME "python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}.zip")
		get_filename_component(PYTHON_ROOT "${Python3_EXECUTABLE}" DIRECTORY)
		file(TO_CMAKE_PATH "${Python3_STDLIB}" SANITIZED_STDLIB)

		MESSAGE(STATUS "Python3_EXECUTABLE=${Python3_EXECUTABLE}")
		MESSAGE(STATUS "Python3_STDLIB=${SANITIZED_STDLIB}")
		
		install(FILES "${Python3_EXECUTABLE}" DESTINATION bin COMPONENT runtime)
		
		# when using python-opencv, it expects python3.dll, not python312.dll
		get_filename_component(VCPKG_TRIPLET_ROOT "${PYTHON_TOOLS_DIR}/../../" ABSOLUTE)
		set(VCPKG_BIN_DIR "${VCPKG_TRIPLET_ROOT}/bin")
		find_file(PYTHON3_STABLE_DLL 
			NAMES python3.dll 
			PATHS "${VCPKG_BIN_DIR}"
			NO_DEFAULT_PATH
		)
		if(PYTHON3_STABLE_DLL)
			message(STATUS "Found python3.dll at: ${PYTHON3_STABLE_DLL}")
			install(FILES "${PYTHON3_STABLE_DLL}" DESTINATION bin COMPONENT runtime)
		endif()

		# install python Lib in python312.zip (without site-packages, which is installed separatly afterwards)
		file(TO_CMAKE_PATH "${Python3_STDLIB}" SANITIZED_STDLIB)
		file(GLOB LIB_CONTENTS RELATIVE "${SANITIZED_STDLIB}" "${SANITIZED_STDLIB}/*")
		list(REMOVE_ITEM LIB_CONTENTS "site-packages")
		install(CODE "
			execute_process(
				COMMAND \"${CMAKE_COMMAND}\" -E tar cf \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/bin/${PYTHON_ZIP_NAME}\" --format=zip -- ${LIB_CONTENTS}
				WORKING_DIRECTORY \"${SANITIZED_STDLIB}\"
			)
		" COMPONENT runtime)
		set(PYTHON_SITE_PACKAGES "${SANITIZED_STDLIB}/site-packages")
		
		install(DIRECTORY "${PYTHON_SITE_PACKAGES}/"
				DESTINATION "${thirdparty_dest_dir}/Lib/site-packages"
				COMPONENT runtime
				PATTERN "*.exe" EXCLUDE
				PATTERN "*.lib" EXCLUDE
				PATTERN "*.hpp" EXCLUDE
				PATTERN "*.h" EXCLUDE
				PATTERN "*/torch/*" EXCLUDE
		)
		if(EXISTS "${PYTHON_SITE_PACKAGES}/torch")
			install(DIRECTORY "${PYTHON_SITE_PACKAGES}/torch"
					DESTINATION "${thirdparty_dest_dir}/Lib/site-packages/"
					COMPONENT runtime
					PATTERN "*.exe" EXCLUDE
					PATTERN "*.lib" EXCLUDE
					PATTERN "*.hpp" EXCLUDE
					PATTERN "*.h" EXCLUDE
					PATTERN "*.dll" EXCLUDE
			)
			file(GLOB_RECURSE PY_DLL_FILES "${PYTHON_SITE_PACKAGES}/torch/*.dll")
			if(PY_DLL_FILES)
				install(FILES ${PY_DLL_FILES} DESTINATION bin COMPONENT runtime)
			endif()
		endif()

		# install python DDLs
		file(GLOB_RECURSE PY_DLL_FILES \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_pyd_dir}/*.dll\")
		install(DIRECTORY "${PYTHON_ROOT}/DLLs"
				DESTINATION "${thirdparty_dest_dir}/"
				COMPONENT runtime
				FILES_MATCHING 
				PATTERN "*.dll"
				PATTERN "*.pyd"
		)

		# install our python scripts in share for convenience
		install(DIRECTORY "${PROJECT_SOURCE_DIR}/corelib/src/python/"
				DESTINATION share
				COMPONENT runtime
				FILES_MATCHING
				PATTERN "*.py"
		)
	ENDIF(WIN32)
  ENDIF(Python3_FOUND)
  
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
		PLUGINS_DIR ${plugin_dest_dir}
		GENERATE_QT_CONF
		NO_TRANSLATIONS
		VERBOSE
	)")
    	IF("${CMAKE_BUILD_TYPE}" STREQUAL "" OR "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appDebug.cmake"
            CONFIGURATIONS Debug
            COMPONENT runtime)
    	ENDIF()
    	IF("${CMAKE_BUILD_TYPE}" STREQUAL "" OR "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
        install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appRelease.cmake"
            CONFIGURATIONS Release
            COMPONENT runtime)
    	ENDIF()
    	IF("${CMAKE_BUILD_TYPE}" STREQUAL "" OR "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
        install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appRelWithDebInfo.cmake"
            CONFIGURATIONS RelWithDebInfo
            COMPONENT runtime)
    	ENDIF()
    	IF("${CMAKE_BUILD_TYPE}" STREQUAL "" OR "${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
        install(SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/deploy_appMinSizeRel.cmake"
            CONFIGURATIONS MinSizeRel
            COMPONENT runtime)
    	ENDIF()
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
        #MESSAGE(STATUS "Qt5 plugin \"${plugin_loc}\" installed in \"${plugin_dest_dir}${plugin_type}\"")
        INSTALL(FILES ${plugin_loc}
          DESTINATION ${plugin_dest_dir}${plugin_type}
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
            #MESSAGE(STATUS "Qt5 plugin \"${plugin_loc}\" installed in \"${plugin_dest_dir}${plugin_type}\"")
        ENDIF(EXISTS ${plugin_loc})
    ENDIF(NOT Qt5Widgets_VERSION VERSION_LESS 5.10.0)
  ELSEIF(QT_PLUGINS_DIR) # Qt4
    # Install needed Qt plugins by copying directories from the qt installation
    # One can cull what gets copied by using 'REGEX "..." EXCLUDE'
    # Exclude debug libraries
    INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats"
        DESTINATION ${plugin_dest_dir}
        COMPONENT runtime 
        REGEX ".*d4.dll" EXCLUDE
        REGEX ".*d4.a" EXCLUDE)
  ENDIF()
 
  IF(Qt5_FOUND OR QT4_FOUND)
      # install a qt.conf file
      # this inserts some cmake code into the install script to write the file
      SET(QT_CONF_FILE [Paths]\nPlugins=plugins)
      IF(APPLE)
        SET(QT_CONF_FILE [Paths]\nPlugins=MacOS/plugins)
      ENDIF(APPLE)
      INSTALL(CODE "
        file(WRITE \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"${QT_CONF_FILE}\")
        " COMPONENT runtime)
  ENDIF()

  # directories to look for dependencies
  SET(DIRS "${QT_LIBRARY_DIRS}" "\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib")
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
    # Glob Qt Plugins
    file(GLOB_RECURSE ALL_LIBS \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    
    if(NOT \"${python_pyd_dir}\" STREQUAL \"\")
        file(GLOB_RECURSE PYD_FILES \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_pyd_dir}/*.pyd\")
        if(PYD_FILES)
            list(APPEND ALL_LIBS \${PYD_FILES})
        endif()
		if(WIN32)
			file(GLOB_RECURSE DLL_FILES \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${python_pyd_dir}/torch/*.dll\")
			if(DLL_FILES)
				list(APPEND ALL_LIBS \${DLL_FILES})
			endif()
		endif()
    endif()
	
    set(BU_CHMOD_BUNDLE_ITEMS ON)
    include(\"BundleUtilities\")
    fixup_bundle(\"${APPS}\" \"\${ALL_LIBS}\" \"${DIRS}\")
  " COMPONENT runtime)
  
ENDIF(BUILD_AS_BUNDLE AND (APPLE OR WIN32))
  
