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

:: pytorch deps
%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe -m pip install numpy packaging "setuptools<82" pyyaml typing_extensions

git config --global core.longpaths true

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
    cd opencv
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
  -DBUILD_opencv_python3=ON ^
  -DPYTHON3_EXECUTABLE=%FINAL_EXPORT_PATH%/installed/%TRIPLET%/tools/python3/python.exe ^
  -DPYTHON3_PACKAGES_PATH=bin/Lib/site-packages ^
  -DBUILD_opencv_java_bindings_generator=OFF ^
  -DWITH_CUDA=ON ^
  -DWITH_VTK=OFF ^
  -DWITH_TBB=ON || exit /b !errorlevel!
cmake --build build --config Release --target install || exit /b !errorlevel!
:: move cv2 package under tools/python3
robocopy "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\bin\Lib" "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib" /E /MOVE /NFL /NDL /NJH /NC /NS /NP
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

:: Set path of unzipped deps
set VCPKG_UNZIPPED_EXPORT_PATH=

set TRIPLET=x64-windows-release
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\bin;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3;%PATH%
set PATH=%CUDA_PATH%\bin;%PATH%
set PATH=%CUDA_PATH%\bin\x64;%PATH%
set PATH=%CUDA_PATH%\extras\CUPTI\lib64;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib\site-packages\torch\lib;%PATH%
set PATH=%VCPKG_UNZIPPED_EXPORT_PATH%\installed\%TRIPLET%\tools\python3\Lib\site-packages\numpy.libs;%PATH%

:: Other dependencies

:: For ZED, modify zed-config.cmake and remove all dependencies
set PATH=%PATH%;%ZED_SDK_ROOT_DIR%\bin

:: For kinect 4 windows SDK v2, move kinect20.dll from system32 to KINECTSDK20_DIR\bin 
:: For kinect 4 windows SDK v1, move kinect10.dll and KinectAudio10.dll to KINECTSDK20_DIR\bin
set PATH=%PATH%;%KINECTSDK20_DIR%\bin

cmake -B build_cuda -GNinja ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_AS_BUNDLE=ON ^
  -DWITH_PYTHON=ON ^
  -DWITH_TORCH=ON ^
  -DWITH_ZED=ON ^
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