@echo off
setlocal enabledelayedexpansion

:: --- CONFIGURATION ---
set "VCPKG_ROOT=%~dp0vcpkg"
set "EXPORT_DIR=%~dp0vcpkg_binaries"
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
    --x-feature=tools ^
	--x-feature=k4w2 ^
	--x-feature=octomap ^
	--x-feature=openmp ^
    --x-feature=realsense2 ^
	--x-feature=openni2 ^
	--x-feature=gtsam-deps ^
	--x-feature=libpointmatcher-deps

:: 3. Export
echo [+] Exporting built binaries to raw folder...
set TARGET_NAME=vcpkg-export-%VCPKG_COMMIT_SHORT%
set TARGET_FULL_PATH=%EXPORT_DIR%\%TARGET_NAME%
if exist "%TARGET_FULL_PATH%" rd /s /q "%TARGET_FULL_PATH%"
"%VCPKG_ROOT%\vcpkg.exe" export --raw --output-dir="%EXPORT_DIR%" --triplet=%TRIPLET%

:: Find the actual exported folder name (it usually contains a date/hash)
for /d %%i in ("%EXPORT_DIR%\vcpkg-export-*") do set "FINAL_EXPORT_PATH=%%i"

ren "%FINAL_EXPORT_PATH%" "%TARGET_NAME%" || exit /b %errorlevel%
set "FINAL_EXPORT_PATH=%TARGET_FULL_PATH%"

:: 4. Other dependencies not in vcpkg
:: libnabo
echo [+] Building libnabo...
if not exist libnabo (
    echo [+] Downloading...
    git clone https://github.com/ethz-asl/libnabo.git
    cd libnabo
    git checkout c925c47
    cd ..
)
cd libnabo
cmake -S . -B build ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DBUILD_SHARED_LIBS=ON ^
  -DLIBNABO_BUILD_DOXYGEN=OFF ^
  -DLIBNABO_BUILD_EXAMPLES=OFF ^
  -DLIBNABO_BUILD_PYTHON=OFF ^
  -DLIBNABO_BUILD_TESTS=OFF || exit /b %errorlevel%
cmake --build build --config Release --target install
cd ..

:: libpointmatcher
echo [+] Building libpointmatcher...
if not exist libpointmatcher (
    echo [+] Downloading and applying patch...
    git clone https://github.com/ethz-asl/libpointmatcher.git
    cd libpointmatcher
    git checkout 7dc58e5
    
    set "PATCH_URL=https://gist.githubusercontent.com/matlabbe/fabab6d0d0e0960dd94e8b633ad713cb/raw/b079c1553041f217246834cb3ddb0847a0c3b96d/pointmatcher_windows_dll.patch"
    curl -L "!PATCH_URL!" -o pointmatcher_windows_dll.patch
    git apply pointmatcher_windows_dll.patch
    cd ..
)
cd libpointmatcher
cmake -S . -B build  ^
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
  -DCMAKE_CXX_FLAGS="-DBOOST_TIMER_ENABLE_DEPRECATED /EHsc -DBOOST_EXCEPTION_DISABLE" || exit /b %errorlevel%
cmake --build build --config Release --target install
cd ..

:: We remove the files in the top-level CMake directory to force use of share/libpointmatcher/cmake
if exist "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\CMake\" (
    rd /s /q "%FINAL_EXPORT_PATH%\installed\%TRIPLET%\CMake"
)

:: gtsam
echo [+] Building gtsam...
if not exist gtsam (
    echo [+] Downloading and applying patch...
    git clone --branch 4.0.0-alpha2 https://github.com/borglab/gtsam.git
    cd gtsam
    
    set "PATCH_URL=https://gist.githubusercontent.com/matlabbe/f66f65540df61edee87d2aa2777e3a73/raw/f6fe0f9e56835fd5a735d954ecdd7453a3036892/gtsam-4.0.0-alpha2-MSVC.patch"
    curl -L "!PATCH_URL!" -o gtsam-4.0.0-alpha2-MSVC.patch
    git apply gtsam-4.0.0-alpha2-MSVC.patch
    cd ..
)
cd gtsam
cmake -S . -B build  ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF  ^
  -DGTSAM_BUILD_TESTS=OFF  ^
  -DGTSAM_BUILD_STATIC_LIBRARY=ON  ^
  -DGTSAM_BUILD_UNSTABLE=OFF  ^
  -DGTSAM_INSTALL_CPPUNILITE=OFF ^
  -DGTSAM_USE_SYSTEM_EIGEN=ON ^
  -DCMAKE_CXX_FLAGS="-DBOOST_TIMER_ENABLE_DEPRECATED -DBOOST_BIND_GLOBAL_PLACEHOLDERS" || exit /b %errorlevel%
cmake --build build --config Release --target install
cd ..

:: opengv
echo [+] Building opengv...
if not exist opengv (
    echo [+] Downloading and applying patch...
    git clone https://github.com/laurentkneip/opengv.git
    cd opengv
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3
    git apply ../patches/opengv_build.patch
    cd ..
)
cd opengv
cmake -S . -B build  ^
  -DVCPKG_MANIFEST_INSTALL=OFF  ^
  -DVCPKG_TARGET_TRIPLET=x64-windows-release  ^
  -DVCPKG_INSTALLED_DIR="%FINAL_EXPORT_PATH%\installed"  ^
  -DCMAKE_TOOLCHAIN_FILE="%FINAL_EXPORT_PATH%\scripts\buildsystems\vcpkg.cmake"  ^
  -DCMAKE_INSTALL_PREFIX="%FINAL_EXPORT_PATH%\installed\%TRIPLET%" ^
  -DBUILD_TESTS=OFF  ^
  -DBUILD_SHARED_LIBS=ON || exit /b %errorlevel%
cmake --build build --config Release --target install
cd ..

:: 5. ZIP the folder
echo [+] Creating final package with 7-Zip...
:: Rip off pdb files
cd /d "%FINAL_EXPORT_PATH%"
del /s /q /f *.pdb

cd ..

set "FINAL_ZIP=%TARGET_NAME%.7z"

:: compress contents without the root folder
"%SEVENZIP_EXE%" a -t7z -mx9 "%FINAL_ZIP%" "%FINAL_EXPORT_PATH%\*"

if %ERRORLEVEL% EQU 0 (
    echo [!] Success! Package created at %FINAL_ZIP%
) else (
    echo [X] 7-Zip failed with error code %ERRORLEVEL%
)