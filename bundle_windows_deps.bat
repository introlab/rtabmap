@echo off
setlocal enabledelayedexpansion

:: --- CONFIGURATION ---
set "VCPKG_ROOT=%~dp0vcpkg"
set "EXPORT_DIR=%~dp0vcpkg_binaries"
:: All deps downloaded/built from source (git clones + the pinned CMake) live here.
set "SRC_DIR=%~dp0vcpkg_deps_from_source"
set "TRIPLET=x64-windows-release"
set "SEVENZIP_EXE=C:\Program Files\7-Zip\7z.exe"

set "VCPKG_JSON=%~dp0vcpkg.json"
for /f "usebackq tokens=*" %%a in (`powershell -NoProfile -Command "(Get-Content '%VCPKG_JSON%' -Raw | ConvertFrom-Json).'builtin-baseline'"` ) do set "VCPKG_COMMIT=%%a"
echo [+] Detected VCPKG baseline commit: %VCPKG_COMMIT%

if "%VCPKG_COMMIT%"=="" (
    echo [X] Error: Could not find 'builtin-baseline' in %VCPKG_JSON%
    pause
    exit /b 1
)

set VCPKG_COMMIT_SHORT=%VCPKG_COMMIT:~0,8%

:: 1. Setup Local vcpkg
if not exist "%VCPKG_ROOT%" (
    echo [+] Local vcpkg not found. Cloning...
    git clone https://github.com/microsoft/vcpkg.git "%VCPKG_ROOT%"
)
pushd "%VCPKG_ROOT%"
git checkout %VCPKG_COMMIT%
call .\bootstrap-vcpkg.bat
popd

:: 2. Install vcpkg dependencies
echo [+] Installing dependencies via vcpkg manifest...
"%VCPKG_ROOT%\vcpkg.exe" install ^
    --triplet=%TRIPLET% ^
    --host-triplet=%TRIPLET% ^
	--clean-after-build ^
    --x-feature=tools ^
    --x-feature=k4w2 ^
    --x-feature=octomap ^
    --x-feature=openmp ^
    --x-feature=realsense2 ^
    --x-feature=openni2 ^
    --x-feature=gtsam-deps ^
    --x-feature=python ^
    --x-feature=libpointmatcher-deps ^
    --x-feature=libfreenect2-deps || exit /b !errorlevel!

:: 3. Export
echo [+] Exporting built binaries to raw folder...
set "VS_LOCATOR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VS_LOCATOR%" -latest -property catalog_productLineVersion`) do set VS_YEAR=vs%%i
set TARGET_NAME=vcpkg-export-%VCPKG_COMMIT_SHORT%-x64-%VS_YEAR%
set TARGET_FULL_PATH=%EXPORT_DIR%\%TARGET_NAME%
if exist "%TARGET_FULL_PATH%" rd /s /q "%TARGET_FULL_PATH%"
"%VCPKG_ROOT%\vcpkg.exe" export --raw --output-dir="%EXPORT_DIR%" --output=%TARGET_NAME% --triplet=%TRIPLET%  || exit /b !errorlevel!
set "FINAL_EXPORT_PATH=%TARGET_FULL_PATH%"

echo [+] Add numpy...
:: We install numpy<2 to be compatible with SuperPoint and SuperGlue scripts
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m ensurepip --upgrade || exit /b %errorlevel%
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m pip install --upgrade pip || exit /b !errorlevel!
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m pip install "numpy<2" || exit /b !errorlevel!

:: 4. Other dependencies not in vcpkg
:: Everything cloned/downloaded below goes into %SRC_DIR% to keep the repo root clean.
if not exist "%SRC_DIR%" mkdir "%SRC_DIR%"
cd /d "%SRC_DIR%"

:: Kinect for Windows SDK v2 (k4w2): rtabmap's FindKinectSDK2.cmake expects the
:: Microsoft SDK layout (KINECTSDK20_DIR\inc and \Lib\x64), but vcpkg's kinectsdk2
:: port installs vcpkg-style (include\ and lib\). Mirror the bundled headers/lib into
:: the expected layout so it is found with KINECTSDK20_DIR pointing at the export.
if "%KINECTSDK20_DIR%"=="" (
    echo Error: KINECTSDK20_DIR environment variable is not set! Install Kinect for Windows SDK v2: https://www.microsoft.com/en-us/download/details.aspx?id=44561
    pause
    exit /b 1
)
echo [+] Arranging Kinect SDK v2 layout for FindKinectSDK2...
set "K4W2_ROOT=%FINAL_EXPORT_PATH%\installed\%TRIPLET%"
robocopy "%K4W2_ROOT%\include" "%K4W2_ROOT%\inc" Kinect*.h Nui*.h /XO >nul
robocopy "%K4W2_ROOT%\lib" "%K4W2_ROOT%\Lib\x64" Kinect20*.lib /XO >nul
robocopy "%KINECTSDK20_DIR%\bin" "%K4W2_ROOT%\bin" Kinect20.dll /XO >nul
:: Robocopy exit codes under 8 mean successful copies/no changes; 8+ is a failure.
if !errorlevel! GEQ 8 (
    echo Error: Robocopy failed with exit code !errorlevel!
    pause
    exit /b !errorlevel!
)

:: orbbec SDK2
echo [+] Building OrbbecSDK_v2...
if not exist OrbbecSDK_v2 (
    echo [+] Downloading and applying patch...
    git clone https://github.com/orbbec/OrbbecSDK_v2.git
    cd OrbbecSDK_v2
    git checkout v2.8.7
    :: Install the CMake package config under lib/cmake/<pkg> so find_package(OrbbecSDK) works
    git apply "%~dp0patches\orbbecsdk_2.8.7.patch"
    cd ..
)
cd OrbbecSDK_v2
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed" ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DOB_BUILD_DOCS=OFF ^
  -DOB_BUILD_EXAMPLES=OFF ^
  -DOB_BUILD_TOOLS=OFF ^
  -DOB_INSTALL_EXAMPLES_SOURCE=OFF || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

if "%K4A_ROOT_DIR%"=="" (
    echo Error: K4A_ROOT_DIR environment variable is not set! Install K4A: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md
    pause
    exit /b 1
)
echo Copying Kinect For Azure components...
robocopy "%K4A_ROOT_DIR%\sdk" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\sdk" /E /XO
:: Robocopy exit codes under 8 mean successful copies/no changes. 
:: 8 or higher means there was a failure.
if !errorlevel! GEQ 8 (
    echo Error: Robocopy failed with exit code !errorlevel!
    pause
    exit /b !errorlevel!
)

:: libnabo
echo [+] Building libnabo...
if not exist libnabo (
    echo [+] Downloading...
    git clone https://github.com/ethz-asl/libnabo.git
    cd libnabo
    :: Jan 27, 2022
    git checkout c925c47
    git apply "%~dp0patches\libnabo_c925c47.patch"
    cd ..
)
cd libnabo
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DSHARED_LIBS=FALSE ^
  -DLIBNABO_BUILD_DOXYGEN=OFF ^
  -DLIBNABO_BUILD_EXAMPLES=OFF ^
  -DLIBNABO_BUILD_PYTHON=OFF ^
  -DLIBNABO_BUILD_TESTS=OFF || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: libpointmatcher
echo [+] Building libpointmatcher...
if not exist libpointmatcher (
    echo [+] Downloading and applying patch...
    git clone https://github.com/ethz-asl/libpointmatcher.git
    cd libpointmatcher
    :: Mar 17, 2023
    git checkout 7dc58e5
    git apply "%~dp0patches\pointmatcher_7dc58e5.patch"
    cd ..
)
cd libpointmatcher
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_TESTS=OFF  ^
  -DBUILD_SHARED_LIBS=ON  ^
  -DPOINTMATCHER_BUILD_EVALUATIONS=OFF ^
  -DPOINTMATCHER_BUILD_EXAMPLES=OFF ^
  -DCMAKE_CXX_FLAGS="-DBOOST_TIMER_ENABLE_DEPRECATED /EHsc -DBOOST_EXCEPTION_DISABLE" || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: We remove the files in the top-level CMake directory to force use of share/libpointmatcher/cmake
if exist "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\CMake\" (
    rd /s /q "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\CMake"
)

:: gtsam
echo [+] Building gtsam...
if not exist gtsam (
    echo [+] Downloading and applying patch...
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    :: June 18, 2025
    git checkout 4.3a0-ros
    git cherry-pick 18af4e6
    git apply "%~dp0patches\gtsam_4_3a0-ros.patch"
    cd ..
)
cd gtsam
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF  ^
  -DGTSAM_BUILD_TESTS=OFF  ^
  -DGTSAM_BUILD_UNSTABLE=OFF  ^
  -DGTSAM_USE_SYSTEM_EIGEN=ON ^
  -DGTSAM_BUILD_WITH_PRECOMPILED_HEADERS=OFF ^
  -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF ^
  -DGTSAM_WITH_EIGEN_MKL=OFF ^
  -DGTSAM_WITH_EIGEN_MKL_OPENMP=OFF ^
  -DCMAKE_CXX_FLAGS="-DBOOST_TIMER_ENABLE_DEPRECATED -DBOOST_BIND_GLOBAL_PLACEHOLDERS" || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: opengv
echo [+] Building opengv...
if not exist opengv (
    echo [+] Downloading and applying patch...
    git clone https://github.com/laurentkneip/opengv.git
    cd opengv
    :: Aug 6, 2020
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3
    git apply "%~dp0patches\opengv_91f4b19c.patch"
    cd ..
)
cd opengv
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DBUILD_TESTS=OFF  ^
  -DBUILD_SHARED_LIBS=ON || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: libfreenect2
echo [+] Building libfreenect2...
if not exist libfreenect2 (
    echo [+] Downloading and applying patch...
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    :: Aug 6, 2021
    git checkout v0.2.1
    :: Patch makes freenect2Config.cmake relocatable so the prebuilt bundle works
    :: after being moved/unzipped, drops cudaDeviceProp::clockRate/computeMode
    :: removed in CUDA 13 from the CUDA build, and renames a CL_ICDL_VERSION local
    :: that collides with the OpenCL 3.0 macro.
    git apply "%~dp0patches\libfreenect2_v0.2.1.patch"
    cd ..
)
cd libfreenect2
:: libfreenect2's FindLibUSB.cmake locates libusb via pkg-config. It declares an
:: ancient cmake_minimum_required (2.8.12), which makes FindPkgConfig default
:: PKG_CONFIG_USE_CMAKE_PREFIX_PATH=OFF, so it ignores the vcpkg CMAKE_PREFIX_PATH.
:: Force it ON so the bundle's lib/pkgconfig (libusb-1.0.pc, libturbojpeg.pc) is
:: searched, use the vcpkg-bundled pkgconf, and also set PKG_CONFIG_PATH as a belt.
set "PKG_CONFIG_PATH=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\lib\pkgconfig"
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DPKG_CONFIG_EXECUTABLE="%FINAL_EXPORT_PATH%\installed\%TRIPLET%\tools\pkgconf\pkgconf.exe" ^
  -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DENABLE_CUDA=OFF ^
  -DENABLE_OPENCL=OFF ^
  -DBUILD_EXAMPLES=OFF ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DBUILD_SHARED_LIBS=ON || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: depthai
echo [+] Building depthai...
if not exist depthai-core (
    echo [+] Downloading...
    git clone https://github.com/luxonis/depthai-core.git
    cd depthai-core
    :: depthai 2.32.0 (Hunter-based; v2 does NOT use vcpkg/CMake presets)
    git checkout v2.32.0
    git submodule update --init --recursive
    cd ..
)
cd depthai-core
:: v2 and its old Hunter sub-deps declare cmake_minimum_required < 3.5, which
:: CMake 4.x rejects, and CMAKE_POLICY_VERSION_MINIMUM does not propagate into
:: Hunter's sub-builds. Use a pinned CMake 3.x just for depthai (Hunter reuses the
:: same cmake binary for its dependency sub-builds).
set "CMAKE3_VER=3.31.12"
set "CMAKE3_NAME=cmake-%CMAKE3_VER%-windows-x86_64"
set "CMAKE3_DIR=%SRC_DIR%\%CMAKE3_NAME%"
set "CMAKE3=%CMAKE3_DIR%\bin\cmake.exe"
if not exist "%CMAKE3_DIR%" (
    echo [+] Downloading CMake %CMAKE3_VER% ...
    curl -L -o "%SRC_DIR%\%CMAKE3_NAME%.zip" "https://github.com/Kitware/CMake/releases/download/v%CMAKE3_VER%/%CMAKE3_NAME%.zip" || exit /b !errorlevel!
    "%SEVENZIP_EXE%" x "%SRC_DIR%\%CMAKE3_NAME%.zip" -o"%SRC_DIR%" -y || exit /b !errorlevel!
)
:: v2 is fully Hunter-managed (no vcpkg toolchain). depthaiDependencies.cmake
:: unconditionally does find_package(OpenCV 4 QUIET CONFIG) (not gated by
:: DEPTHAI_OPENCV_SUPPORT); if it finds the bundle's OpenCV, OpenCVModules does
:: find_dependency(TIFF) which collides with Hunter's config-less FindTIFF. We don't
:: need depthai's OpenCV at all (CameraDepthAI converts ImgFrame->cv::Mat itself), so
:: hard-disable the lookup. DEPTHAI_ENABLE_CURL=OFF avoids Hunter's old zlib/curl.
"%CMAKE3%" -S . -B build -GNinja ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_SHARED_LIBS=ON ^
  -DDEPTHAI_OPENCV_SUPPORT=OFF ^
  -DCMAKE_DISABLE_FIND_PACKAGE_OpenCV=ON ^
  -DDEPTHAI_ENABLE_CURL=OFF ^
  -DDEPTHAI_BUILD_EXAMPLES=OFF ^
  -DDEPTHAI_BUILD_TESTS=OFF || exit /b !errorlevel!
"%CMAKE3%" --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: CCCoreLib
echo [+] Building CCCoreLib...
if not exist CCCoreLib (
    echo [+] Downloading...
    git clone https://github.com/CloudCompare/CCCoreLib.git
    cd CCCoreLib
    :: June 20, 2026
    git checkout 4095bea6552096cb528c3b8b5cb9505df2aa6002
    git submodule update --init --recursive
    :: nanoflann is an internal dep (only used in src/Kriging.cpp, no public header),
    :: but CCCoreLib links it PUBLIC, leaking nanoflann::nanoflann into the exported
    :: interface and breaking find_package(CCCoreLib). Link it PRIVATE instead.
    git apply "%~dp0patches\cccorelib_4095bea.patch"
    cd ..
)
cd CCCoreLib
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCCCORELIB_SHARED=ON ^
  -DCCCORELIB_USE_CGAL=OFF ^
  -DCCCORELIB_USE_TBB=ON ^
  -DCCCORELIB_USE_QT_CONCURRENT=OFF || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..

:: 5. ZIP the folder
echo [+] Creating final package with 7-Zip...
:: Rip off pdb files
cd /d "%FINAL_EXPORT_PATH%"
del /s /q /f *.pdb >nul 2>&1

cd ..

set "FINAL_ZIP=%TARGET_NAME%.7z"

:: compress contents without the root folder
"%SEVENZIP_EXE%" u -t7z -mx9 "%FINAL_ZIP%" "%FINAL_EXPORT_PATH%\*" -up0q0

if !errorlevel! EQU 0 (
    echo [!] Success! Package created at %FINAL_ZIP%
) else (
    echo [X] 7-Zip failed with error code !errorlevel!
)


:: Example building rtabmap afterwards
goto :EndComment

:: The vcpkg export folder should not be in the rtabmap source directory
:: (otherwise we get some cmake errors about that)
set VCPKG_UNZIPPED_EXPORT_PATH=%USERPROFILE%\Downloads\vcpkg-export-########-x64-vs2022

set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release\bin;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release\sdk\windows-desktop\amd64\release\bin;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release\tools\python3\Lib\site-packages\numpy.libs;%PATH%
set K4A_ROOT_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release
set KINECTSDK20_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release

cmake -B build -GNinja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_AS_BUNDLE=ON ^
  -DWITH_PYTHON=ON ^
  -DWITH_ZED=OFF ^
  -DWITH_CERES=ON ^
  -DWITH_ORBBEC_SDK=ON ^
  -DWITH_FREENECT2=ON ^
  -DWITH_K4W2=ON ^
  -DWITH_K4A=ON ^
  -DWITH_DEPTHAI=ON ^
  -DWITH_REALSENSE2=ON ^
  -DWITH_CCCORELIB=ON ^
  -DWITH_PDAL=OFF ^
  -DVCPKG_MANIFEST_INSTALL=OFF ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release ^
  -DVCPKG_INSTALLED_DIR="%VCPKG_UNZIPPED_EXPORT_PATH%/installed" ^
  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_UNZIPPED_EXPORT_PATH%/scripts/buildsystems/vcpkg.cmake ^
  -DGTSAM_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release\CMake 

cmake --build build --config Release --target package

:: To install CPU pytorch inside rtabmap package afterwards.
:: Note that python.exe is the one in the bin directory of the package, not the system one.
python.exe -m pip install torch torchvision opencv-python-headless "numpy<2"

:EndComment
