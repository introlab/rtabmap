@echo off
setlocal enabledelayedexpansion

IF NOT DEFINED CUDA_PATH (
    echo [ERROR] CUDA_PATH is not set.
    exit /b 1
)

set PATH=%CUDA_PATH%\bin;%PATH%
set PATH=%CUDA_PATH%\bin\x64;%PATH%
set PATH=%CUDA_PATH%\extras\CUPTI\lib64;%PATH%

:: CUDA Toolkit should be manually installed on the computer before running this script
:: We assume also that cuDNN is merged into CUDA installed directory.
where nvcc >nul 2>&1
if !errorlevel! neq 0 (
    echo [ERROR] nvcc was not found in your PATH.
    pause
    exit /b
)
for /f "tokens=5" %%a in ('nvcc --version ^| findstr "release"') do (
    set "RAW_VER=%%a"
    :: This removes the trailing comma
    set "CUDA_VER=!RAW_VER:,=!"
	set "CUDA_VER_SHORT=!CUDA_VER:.=!"
)
if "!CUDA_VER!"=="" (
    echo [ERROR] Could not parse CUDA version.
    pause
    exit /b
)
echo Installed CUDA Toolkit: %CUDA_VER%

:: --- CONFIGURATION ---
set "VCPKG_ROOT=%~dp0vcpkg"
set "EXPORT_DIR=%~dp0vcpkg_binaries"
:: All deps downloaded/built from source (pytorch, torchvision, opencv, libfreenect2, ...) live here.
set "SRC_DIR=%~dp0vcpkg_deps_from_source"
set "TRIPLET=x64-windows-release"
set "SEVENZIP_EXE=C:\Program Files\7-Zip\7z.exe"

set "VCPKG_JSON=%~dp0vcpkg.json"
for /f "usebackq tokens=*" %%a in (`powershell -NoProfile -Command "(Get-Content '%VCPKG_JSON%' -Raw | ConvertFrom-Json).'builtin-baseline'"` ) do set "VCPKG_COMMIT=%%a"

if "%VCPKG_COMMIT%"=="" (
    echo [X] Error: Could not find 'builtin-baseline' in %VCPKG_JSON%
    pause
    exit /b 1
)

set VCPKG_COMMIT_SHORT=%VCPKG_COMMIT:~0,8%
set "VS_LOCATOR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VS_LOCATOR%" -latest -property catalog_productLineVersion`) do set VS_YEAR=vs%%i
set ORG_TARGET_NAME=vcpkg-export-%VCPKG_COMMIT_SHORT%-x64-%VS_YEAR%
set TARGET_NAME=%ORG_TARGET_NAME%-cuda%CUDA_VER_SHORT%
set VCPKG_EXPORT_PATH=%EXPORT_DIR%\%ORG_TARGET_NAME%
set FINAL_EXPORT_PATH=%EXPORT_DIR%\%TARGET_NAME%

if not exist "%FINAL_EXPORT_PATH%" (
    if not exist "%VCPKG_EXPORT_PATH%" (
        call bundle_windows_deps.bat || exit /b !errorlevel!
	)
	echo [+] Copying %VCPKG_EXPORT_PATH% to %FINAL_EXPORT_PATH%
	xcopy "%VCPKG_EXPORT_PATH%" "%FINAL_EXPORT_PATH%\" /E /I /H /Y /Q || exit /b !errorlevel!
	echo [+] Remove opencv built by vcpkg
	rmdir /s /q "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\include\opencv4" || exit /b !errorlevel!
	rmdir /s /q "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\share\opencv4" || exit /b !errorlevel!
	rmdir /s /q "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\share\opencv" || exit /b !errorlevel!
	del "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\bin\opencv*" || exit /b !errorlevel!
	del "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\lib\opencv*" || exit /b !errorlevel!
	:: bundle cudnn runtime libraries
	xcopy "%CUDA_PATH%\bin\x64\cudnn*.dll" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\bin\" /Y
)

:: ZED: copy the main library + the zed-config.cmake so find_package(ZED) resolves
:: it from the export (the config derives paths from ZED_SDK_ROOT_DIR/CMAKE_PREFIX_PATH).
if "%ZED_SDK_ROOT_DIR%"=="" (
    echo Error: ZED_SDK_ROOT_DIR environment variable is not set!
    pause
    exit /b 1
)
echo Copying ZED main library only...
:: Only the core runtime goes into the bundle; the neural-depth/TensorRT DLLs are
:: shipped separately (see the zed-neural-extra archive below).
robocopy "%ZED_SDK_ROOT_DIR%\bin" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\bin" sl_zed64.dll zlibwapi.dll /XO
robocopy "%ZED_SDK_ROOT_DIR%\lib" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\lib" /E /XO
robocopy "%ZED_SDK_ROOT_DIR%\include" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\include" /E /XO
:: Install ZED's CMake config at the export prefix root so find_package(ZED 2) (config
:: mode, via the vcpkg toolchain CMAKE_PREFIX_PATH) finds it.
robocopy "%ZED_SDK_ROOT_DIR%" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%" zed-config.cmake zed-config-version.cmake /XO
:: Robocopy exit codes under 8 mean successful copies/no changes.
:: 8 or higher means there was a failure.
if !errorlevel! GEQ 8 (
    echo Error: Robocopy failed with exit code !errorlevel!
    pause
    exit /b !errorlevel!
)
:: Archive the remaining ZED runtime DLLs (neural-depth / TensorRT extras) as a
:: separate optional download, kept out of the main bundle to keep it small. Built
:: only once: skipped if the archive already exists. Staged with robocopy /XF
:: (excludes by name; avoids 7z's "!" exclude switch that delayed expansion mangles).
set "ZED_EXTRA_7Z=%EXPORT_DIR%\%TARGET_NAME%-zed-neural-extra.7z"
set "ZED_EXTRA_DIR=%SRC_DIR%\zed-neural-extra"
if not exist "%ZED_EXTRA_7Z%" (
    echo [+] Creating ZED neural extra archive...
    if exist "%ZED_EXTRA_DIR%" rd /s /q "%ZED_EXTRA_DIR%"
    robocopy "%ZED_SDK_ROOT_DIR%\bin" "%ZED_EXTRA_DIR%" *.dll /XF sl_zed64.dll zlibwapi.dll >nul
    "%SEVENZIP_EXE%" a -t7z -mx9 "%ZED_EXTRA_7Z%" "%ZED_EXTRA_DIR%\*" || exit /b !errorlevel!
)

:: pytorch deps
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m pip install numpy packaging "setuptools<82" pyyaml typing_extensions

git config --global core.longpaths true

:: All deps cloned/built below go into %SRC_DIR% (incl. libfreenect2 built by
:: bundle_windows_deps.bat) to keep the repo root clean.
if not exist "%SRC_DIR%" mkdir "%SRC_DIR%"
cd /d "%SRC_DIR%"

:: pytorch, build with local cuda libraries to avoid duplicating them when we install rtabmap
echo [+] Building pytorch with cuda support...
if not exist pytorch (
    echo [+] Downloading pytorch...
    git clone https://github.com/pytorch/pytorch || exit /b !errorlevel!
    cd pytorch
	:: Jan 21, 2026
    git checkout v2.10.0 
	git submodule update --init --recursive || exit /b !errorlevel!
    cd ..
)

set "PYTHONHOME=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\tools\python3"
set "Python_ROOT_DIR=%FINAL_EXPORT_PATH%\installed\%TRIPLET%"
set CMAKE_GENERATOR=Ninja
set BUILD_TEST=0
set ATEN_NO_TEST=1
set INSTALL_TEST=OFF
set "LIB=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\lib;%LIB%"
set "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\include;INCLUDE=%INCLUDE%"

:: check if torch is installed
%PYTHONHOME%/python.exe -m pip show torch >nul 2>&1
if !errorlevel! neq 0 (
    cd pytorch
    %PYTHONHOME%/python.exe setup.py install || exit /b !errorlevel!
    cd ..
)
if exist "%PYTHONHOME%\Lib\site-packages\torch\test" rd /s /q %PYTHONHOME%\Lib\site-packages\torch\test"
del "%PYTHONHOME%\Lib\site-packages\torch\bin\test_*" || exit /b !errorlevel!

echo [+] Building torchvision...
if not exist torchvision (
    echo [+] Downloading torchvision...
    git clone https://github.com/pytorch/vision.git torchvision || exit /b !errorlevel!
    cd torchvision
	:: Jan 6, 2026
    git checkout v0.25.0
	git submodule update --init --recursive || exit /b !errorlevel!
    cd ..
)
cd torchvision
set PATH=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\bin;%PATH%
set DISTUTILS_USE_SDK=1
set TORCHVISION_INCLUDE=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\include
set TORCHVISION_LIBRARY=%FINAL_EXPORT_PATH%\installed\%TRIPLET%\lib
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m pip install . -v --no-build-isolation || exit /b !errorlevel!
cd ..


:: opencv_cuda
echo [+] Building opencv with cuda support...
if not exist opencv (
    echo [+] Downloading opencv...
    git clone https://github.com/opencv/opencv.git || exit /b !errorlevel!
    cd opencv
    :: 4.13.0 minimum required to be compatible with cuda 13
	:: Dec 31, 2025
    git checkout 4.13.0
    cd ..
)
if not exist opencv_contrib (
    echo [+] Downloading opencv_contrib...
	git clone https://github.com/opencv/opencv_contrib.git || exit /b !errorlevel!
    cd opencv_contrib
    :: 4.13.0 minimum required to be compatible with cuda 13
	:: Dec 31, 2025
    git checkout 4.13.0
    cd ..
)
cd opencv
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=%TRIPLET%  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DOPENCV_BIN_INSTALL_PATH="bin" ^
  -DOPENCV_LIB_INSTALL_PATH="lib" ^
  -DOPENCV_CONFIG_INSTALL_PATH="share/opencv" ^
  -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ^
  -DBUILD_SHARED_LIBS=ON ^
  -DBUILD_TESTS=OFF ^
  -DBUILD_PERF_TESTS=OFF ^
  -DOPENCV_ENABLE_NONFREE=ON ^
  -DBUILD_opencv_apps=OFF ^
  -DBUILD_opencv_cudacodec=OFF ^
  -DBUILD_opencv_python3=OFF ^
  -DBUILD_opencv_python_bindings_generator=OFF ^
  -DBUILD_opencv_python_tests=OFF ^
  -DBUILD_opencv_java_bindings_generator=OFF ^
  -DWITH_CUDA=ON ^
  -DWITH_VTK=OFF ^
  -DWITH_TBB=ON || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
cd ..


:: freenect2 with cuda support
:: libfreenect2's CUDA processors include <helper_math.h>, which modern CUDA toolkits
:: no longer ship (it moved to the NVIDIA/cuda-samples repo). Fetch it and point
:: NVCUDASAMPLES_ROOT at it; libfreenect2 adds %NVCUDASAMPLES_ROOT%\common\inc to nvcc.
set "CUDA_SAMPLES_DIR=%SRC_DIR%\cuda-samples"
if not exist "%CUDA_SAMPLES_DIR%\common\inc\helper_math.h" (
    mkdir "%CUDA_SAMPLES_DIR%\common\inc"
    curl -L -o "%CUDA_SAMPLES_DIR%\common\inc\helper_math.h" "https://raw.githubusercontent.com/NVIDIA/cuda-samples/v12.5/Common/helper_math.h" || exit /b !errorlevel!
)
set "NVCUDASAMPLES_ROOT=%CUDA_SAMPLES_DIR%"
cd libfreenect2
cmake -S . -B build_cuda -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DPKG_CONFIG_EXECUTABLE="%FINAL_EXPORT_PATH%\installed\%TRIPLET%\tools\pkgconf\pkgconf.exe" ^
  -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DENABLE_CUDA=ON ^
  -DENABLE_OPENCL=ON ^
  -DENABLE_OPENGL=ON ^
  -DBUILD_EXAMPLES=OFF ^
  -DBUILD_SHARED_LIBS=ON || exit /b !errorlevel!
cmake --build build_cuda --config Release --target install || exit /b !errorlevel!
cd ..

:: CudaSift (CUDA SIFT/SURF GPU features, matlabbe fork; needs CUDA)
echo [+] Building CudaSift...
if not exist CudaSift (
    echo [+] Downloading...
    git clone https://github.com/matlabbe/CudaSift.git
    cd CudaSift
    git checkout f764e14ae59ee78ff5b282d38790301d80faadc3
    cd ..
)
cd CudaSift
cmake -S . -B build -GNinja ^
  -DVCPKG_MANIFEST_INSTALL=OFF ^
  -DVCPKG_TARGET_TRIPLET=%TRIPLET% ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed" ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DVERBOSE=OFF ^
  -DBUILD_SHARED_LIBS=ON || exit /b !errorlevel!
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
"%SEVENZIP_EXE%" u -t7z -mx9 "%FINAL_ZIP%" "%FINAL_EXPORT_PATH%\*" -up0q0 || exit /b !errorlevel!

if !errorlevel! EQU 0 (
    echo [!] Success! Package created at %FINAL_ZIP%
) else (
    echo [X] 7-Zip failed with error code !errorlevel!
)

:: Example building rtabmap with opencv cuda and libtorch afterwards
goto :EndComment

:: The vcpkg export folder should not be in the rtabmap source directory
:: (otherwise we get some cmake errors about that)
set VCPKG_UNZIPPED_EXPORT_PATH=%USERPROFILE%\Downloads\vcpkg-export-########-x64-vs2022

set TRIPLET=x64-windows-release
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\bin;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3;%PATH%
set PATH=%CUDA_PATH%\bin;%PATH%
set PATH=%CUDA_PATH%\bin\x64;%PATH%
set PATH=%CUDA_PATH%\extras\CUPTI\lib64;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib\site-packages\torch\lib;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib\site-packages\numpy.libs;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\x64-windows-release\sdk\windows-desktop\amd64\release\bin;%PATH%
set K4A_ROOT_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%
set KINECTSDK20_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%
set ZED_SDK_ROOT_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%

cmake -B build_cuda -GNinja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_AS_BUNDLE=ON ^
  -DWITH_PYTHON=ON ^
  -DWITH_TORCH=ON ^
  -DWITH_ZED=ON ^
  -DWITH_CUDASIFT=ON ^
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
  -DVCPKG_TARGET_TRIPLET=%TRIPLET% ^
  -DVCPKG_INSTALLED_DIR="%VCPKG_UNZIPPED_EXPORT_PATH%/installed" ^
  -DCMAKE_TOOLCHAIN_FILE=%VCPKG_UNZIPPED_EXPORT_PATH%/scripts/buildsystems/vcpkg.cmake ^
  -DGTSAM_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\CMake ^
  -DTorch_DIR=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib\site-packages\torch\share\cmake\Torch

cmake --build build_cuda --config Release --target package

:: Generate superpoint weights (from share directory of the installed package)
curl -L -O "https://raw.githubusercontent.com/magicleap/SuperPointPretrainedNetwork/master/demo_superpoint.py"
curl -L -O "https://github.com/magicleap/SuperPointPretrainedNetwork/raw/refs/heads/master/superpoint_v1.pth"
..\bin\python.exe rtabmap_trace_superpoint.py

:EndComment