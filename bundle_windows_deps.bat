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
set "VS_LOCATOR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VS_LOCATOR%" -latest -property catalog_productLineVersion`) do set VS_YEAR=vs%%i
set TARGET_NAME=vcpkg-export-%VCPKG_COMMIT_SHORT%-x64-%VS_YEAR%
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
	:: Jan 27, 2022
    git checkout c925c47
	git apply ../patches/libnabo_c925c47.patch
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
  -DSHARED_LIBS=FALSE ^
  -DLIBNABO_BUILD_DOXYGEN=OFF ^
  -DLIBNABO_BUILD_EXAMPLES=OFF ^
  -DLIBNABO_BUILD_PYTHON=OFF ^
  -DLIBNABO_BUILD_TESTS=OFF || exit /b %errorlevel%
cmake --build build --config Release --target install || exit /b %errorlevel%
cd ..

:: libpointmatcher
echo [+] Building libpointmatcher...
if not exist libpointmatcher (
    echo [+] Downloading and applying patch...
    git clone https://github.com/ethz-asl/libpointmatcher.git
    cd libpointmatcher
	:: Mar 17, 2023
    git checkout 7dc58e5
    git apply ../patches/pointmatcher_7dc58e5.patch
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
cmake --build build --config Release --target install || exit /b %errorlevel%
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
	:: Feb 28, 2026
	git checkout f7dddec7b8766e97c1c04894da096d304bbdda92
    git apply ../patches/gtsam_f7dddec7.patch
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
  -DGTSAM_BUILD_UNSTABLE=OFF  ^
  -DGTSAM_USE_SYSTEM_EIGEN=ON ^
  -DGTSAM_BUILD_WITH_PRECOMPILED_HEADERS=OFF ^
  -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF ^
  -DGTSAM_WITH_EIGEN_MKL=ON ^
  -DGTSAM_WITH_EIGEN_MKL_OPENMP=ON ^
  -DCMAKE_CXX_FLAGS="-DBOOST_TIMER_ENABLE_DEPRECATED -DBOOST_BIND_GLOBAL_PLACEHOLDERS" || exit /b %errorlevel%
cmake --build build --config Release --target install || exit /b %errorlevel%
cd ..

:: opengv
echo [+] Building opengv...
if not exist opengv (
    echo [+] Downloading and applying patch...
    git clone https://github.com/laurentkneip/opengv.git
    cd opengv
	:: Aug 6, 2020
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3
    git apply ../patches/opengv_91f4b19c.patch
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
cmake --build build --config Release --target install || exit /b %errorlevel%
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

if %ERRORLEVEL% EQU 0 (
    echo [!] Success! Package created at %FINAL_ZIP%
) else (
    echo [X] 7-Zip failed with error code %ERRORLEVEL%
)