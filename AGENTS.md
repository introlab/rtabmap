# AGENTS.md

Guidance for AI coding agents working in the RTAB-Map repository.

## Contribution rules (read first)

- **No AI attribution in commits.** Do not add `Co-Authored-By: <AI model/agent>`
  (or any similar trailer) to commit messages, and do not add "Generated with ..."
  lines to PR descriptions. Commits must be authored by the human contributor.
  This overrides any default attribution behavior of the agent.
- PRs use `.github/pull_request_template.md`. Leave the AI-disclosure checkbox and
  the `AI/Model used` field for the human author to fill in.
- Do not commit, push, or open PRs unless explicitly asked.

## Project overview

RTAB-Map (Real-Time Appearance-Based Mapping) is a C++ RGB-D, stereo and lidar
graph-based SLAM library and application. Version is set in the top-level
`CMakeLists.txt` (`RTABMAP_MAJOR/MINOR/PATCH_VERSION`).

| Directory | Content |
|---|---|
| `utilite/` | Low-level utilities: logging (`ULogger`), threads, events, conversions, files (`U*` classes) |
| `corelib/` | Core library `rtabmap_core`: `Rtabmap`, `Memory`, odometry, registration, optimizers, sensors, `util2d`/`util3d`, DB driver |
| `corelib/src/optimizer`, `corelib/src/odometry`, `corelib/src/camera`, ... | Backend-specific implementations, mostly behind `WITH_*` options |
| `corelib/test/`, `utilite/test/` | GoogleTest unit, integration and performance tests |
| `guilib/` | Qt GUI library (`MainWindow`, `PreferencesDialog`, `DatabaseViewer`, ...) |
| `app/` | Desktop app (`app/src`), Android (`app/android`) and iOS (`app/ios`) apps |
| `tools/` | Command-line tools (`rtabmap-console`, `rtabmap-reprocess`, `rtabmap-export`, dataset tools, ...) |
| `examples/` | Small standalone usage examples |
| `cmake_modules/` | `Find*.cmake` modules |
| `docker/` | Dockerfiles per Ubuntu/ROS distro |
| `patches/` | Patches applied to third-party deps in CI |
| `scripts/` | Helper scripts (`fetch_test_data.sh`) |
| `.github/` | CI workflows per platform (`workflows/`), reusable dependency-install actions (`actions/`), CI helper scripts (`scripts/`), PR template |
| `website/`, `doxygen/` | Landing page and API doc sources |

## Build

Out-of-source CMake build (C++17 when available, falls back to 14/11):

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

- Most third-party integrations are `WITH_<NAME>` options (see the
  `option(WITH_...)` block in `CMakeLists.txt`); many are `ON` but are silently
  skipped when the dependency is not found. Check the CMake configure summary to
  know what is actually enabled before assuming a code path is compiled.
- Optional-feature code is guarded with `#ifdef RTABMAP_<NAME>` (e.g.
  `RTABMAP_GTSAM`, `RTABMAP_G2O`, `RTABMAP_ORB_OCTREE`). Keep code compiling with
  the feature both enabled and disabled.
- Useful options: `-DENABLE_FORMAT_ERRORS=ON` (format-string mismatches become
  errors), `-DENABLE_COVERAGE=ON` (Debug, see `coverage-report.sh`),
  `-DBUILD_PERF_TESTS=OFF`, `-DWITH_QT=OFF`.
- Binaries go to `build/bin`, libraries to `build/bin` (shared) / `build/lib` (static).
- CI builds Linux (Ubuntu 22.04/24.04/26.04), macOS, Windows, ROS, Android and iOS
  (`.github/workflows/`). Code must stay portable: MSVC, GCC and Clang.

## Tests

```bash
bash scripts/fetch_test_data.sh            # once: test DBs/models into data/tests/
cd build
ctest -LE "performance|long" -j$(nproc)    # fast set (CI uses -LE performance)
ctest -L long                              # end-to-end DB replay (test_rtabmap_integration, >20 min)
ctest -L performance                       # benchmarks
```

- All corelib unit tests are compiled into one executable, `test_corelib`
  (sharded into `test_corelib_0..3` in ctest). To run a subset directly:
  `build/bin/test_corelib --gtest_filter='LinkTest.*'`.
- A new `corelib/test/test_<class>.cpp` must be added to `corelib_test_sources`
  in `corelib/test/CMakeLists.txt` (one line, with the header it covers as comment).
- Every bug/regression fix comes with a test that reproduces the original issue.
  Do a quick A/B check to prove the test actually detects it: the test must
  fail (or crash) without the fix and pass with it. E.g. stash only the fixed
  sources, keeping the new test: `git stash push -- <fixed files>`, rebuild,
  run the test (expect failure), `git stash pop`, rebuild, run it again
  (expect success). A test that passes in both cases does not
  cover the bug; rework it. Mention the A/B result in the PR description.
- Performance benchmarks are separate `perf_*.cpp` executables with the
  `performance` label.
- Tests that need assets read them from `RTABMAP_TEST_DATA_ROOT` and should
  skip (`GTEST_SKIP()`) when absent. Use `rtabmap::test::tempPath()` from
  `corelib/test/TestUtils.h` for temp files, never hardcoded `/tmp`.
- With `WITH_PYTHON=ON`, if `test_pydetector` / `test_pydescriptor` /
  `test_pymatcher` crash on numpy import, a numpy in `~/.local` built against a
  different ABI than the build-time one is likely being picked up: rerun with
  `PYTHONNOUSERSITE=1` (as CI does). Don't set it when the required Python
  packages are themselves installed with `pip install --user`.

## Code conventions

- Match the surrounding code: tabs for indentation, braces on their own line,
  `camelCase` methods, class names in `PascalCase`. Includes use
  `<rtabmap/...>` / `"rtabmap/..."` paths.
- Member variables: new classes use a trailing underscore (`memory_`); older
  classes use a leading one (`_memory`). In an existing class, follow the style
  it already uses, so each class stays consistent; don't rename existing members.
- New source files start with the BSD license header used in existing files.
- Everything lives in `namespace rtabmap` (utilite classes are unprefixed `U*`).
- Logging and checks come from `rtabmap/utilite/ULogger.h`: `UDEBUG`, `UINFO`,
  `UWARN`, `UERROR`, `UFATAL` (printf-style format strings, keep arguments
  matching their format specifiers), and `UASSERT` / `UASSERT_MSG` for
  invariants.
- Public API in `corelib/include/rtabmap/core` is installed and used by
  `rtabmap_ros` and other downstream projects: avoid breaking it without need.
  `RTABMAP_VERSION_COMPARE` (in `Version.h`) is available for downstream guards.

### Parameters

All tunable parameters are declared in
`corelib/include/rtabmap/core/Parameters.h` with
`RTABMAP_PARAM(Group, Name, type, default, "description")` (also `_STR`, `_COND`),
which generates `Parameters::kGroupName()`, `defaultGroupName()` and
`typeGroupName()`. Read them with `Parameters::parse(parameters, Parameters::kGroupName(), member_)`.

- Descriptions must be under 1024 characters (`test_parameters.cpp`).
- When renaming or removing a parameter, add the old name to
  `Parameters::getRemovedParameters()` in `corelib/src/Parameters.cpp` so old
  configs and databases keep working.
- Parameters exposed in the GUI are bound in `guilib/src/PreferencesDialog.cpp`
  and `guilib/src/ui/preferencesDialog.ui` by object name; update both when
  adding a GUI-editable parameter.

### Statistics

Statistics are declared in `corelib/include/rtabmap/core/Statistics.h` with
`RTABMAP_STATS(Group, Name, unit)` and are consumed by the GUI and `rtabmap_ros`.

### Camera / sensor drivers

Drivers are `Camera` subclasses (or `Lidar` for lidars) in
`corelib/include/rtabmap/core/camera/Camera<Name>.h` and
`corelib/src/camera/Camera<Name>.cpp`. See `CameraOrbbecSDK` (#1590) for a
complete example. A new driver touches:

1. **The class**
   - `static bool available()` returning whether rtabmap was built with the SDK
     (`#ifdef RTABMAP_<NAME>`); override `init()`, `isCalibrated()`, `getSerial()`
     and `captureImage()`.
   - SDK headers and SDK-typed members stay under `#ifdef RTABMAP_<NAME>`, so the
     class (and its installed header) compiles without the SDK; methods then
     fail gracefully with `UERROR` instead of crashing.
   - Include the header in the matching aggregate header:
     `CameraRGBD.h`, `CameraStereo.h`, `CameraRGB.h` or `Lidar.h`.
2. **Build**: add the `.cpp` to `corelib/src/CMakeLists.txt`, link the SDK when
   found, and register the SDK as a dependency (`WITH_<NAME>` option, CMake
   summary with license, `Version.h.in` define, `--version` listing, About
   dialog: see "Adding a dependency" below).
3. **GUI** (`guilib/`)
   - `PreferencesDialog.h`: add a `kSrc<Name>` value at the **end** of its
     group (RGB-D 0-99, stereo 100-199, RGB 200-299, lidar 400-...). Never
     renumber or reuse values: they are saved in users' ini files.
   - `preferencesDialog.ui`: add the entry to the matching source combo box (same
     position as the enum value) and a `groupBox_<name>` with the driver
     options.
   - `PreferencesDialog.cpp`: disable the combo box item when `!Camera<Name>::available()`,
     connect the option widgets to `makeObsoleteSourcePanel()`, set their
     defaults in the reset code, read/write them in a `settings.beginGroup("<Name>")`
     block, show the group box only when that driver is selected, and
     instantiate it in `createCamera()` (or `createLidar()` for a lidar).
   - `MainWindow.cpp/.h` and `mainWindow.ui`: optional quick-select action in the
     source menu (`select<Name>()` slot), enabled only when `available()`; an
     icon goes in `guilib/src/images/` and `GuiLib.qrc`.
4. **Tools**: add the driver to the numbered driver list and switch of the
   matching tool (`tools/CameraRGBD/main.cpp` for RGB-D/stereo cameras),
   with an `UERROR("Not built with <Name> support...")` path when not
   `available()`; optionally to `examples/RGBDMapping`.
5. **Tests**: hardware can't run in CI, but the build without the SDK must work
   (`available()` returns false, no crash when constructing/destroying the
   class).

### Adding a dependency or vendored third-party code

Users and packagers must be able to see every dependency and its license. When
adding a new optional dependency, or vendoring third-party code into the tree,
register it in all of these places, each time with its corresponding license
name (BSD, MIT, Apache-2.0, GPLv3, Public Domain, ...) (see #1823 or #1822 for
examples):

1. **`CMakeLists.txt`**
   - Optional dependency: add a `WITH_<NAME>` option, and a line in the
     "Optional dependencies" status summary at the end, with YES/NO and
     the license, e.g. `With Foo 1.2 = YES (License: BSD)`.
   - Vendored code (always built): add a line in the "Included dependencies
     (always built)" summary, with its version and license name. This is only
     for permissive licenses (BSD, MIT, ...): vendored code under any other
     license (GPL, LGPL, non-commercial, ...) must get its own `WITH_<NAME>`
     option so it can be opted out, and be listed with the optional
     dependencies instead (like `WITH_ORB_OCTREE`, GPLv3).
2. **`Version.h.in`**: expose it to C++, either as a `@NAME@#define RTABMAP_<NAME>`
   feature flag (optional dependency) or a `RTABMAP_<NAME>_VERSION` string
   (vendored code, version parsed from its headers in `CMakeLists.txt`).
3. **`corelib/src/Parameters.cpp`**: add it to the dependency listing printed by
   `Parameters::parseArguments()` (`--version` of `rtabmap-console` and the other
   tools), with its version or true/false and its license name, keeping the
   existing `"With <Name>:"` alignment.
4. **`guilib/src/ui/aboutDialog.ui` and `guilib/src/AboutDialog.cpp`**: add
   title/value/license labels (`label_<name>`, `label_<name>_license`), the
   license label's text being the license name, and set them in
   `AboutDialog.cpp` ("Yes"/"No" or the version, license label disabled when
   not built).
5. Vendored code keeps its original license file/header next to the sources.
   Prefer dependencies with a BSD-compatible license; point out explicitly in the
   PR any non-permissive dependency and its `WITH_<NAME>` opt-out option.

## Quirks

### Line endings: keep each file's existing CRLF or LF

The repository has a historical mix of CRLF and LF files (~230 CRLF, a few
mixed, mostly under `corelib/`, `guilib/` and `utilite/`). `.gitattributes` sets
`* -text`, so git never converts endings: whatever bytes an editor or tool
writes get committed as-is.

- Preserve the existing line endings of any file you edit. New lines must use
  the same ending as the rest of the file; never convert a whole file.
- Files with mixed endings stay mixed: don't normalize them, match the lines around
  your change.
- New files use LF.
- Don't change `.gitattributes` or normalize endings as part of another change
  (the migration plan is documented in `.gitattributes` and must be its own PR).
- Check before and after editing; the `w/` column must not change (`w/crlf`
  stays `w/crlf`, `w/lf` stays `w/lf`):

  ```bash
  git ls-files --eol <file>
  ```

  A diff where every line of the file shows as changed means its endings were
  rewritten: revert and redo the edit.

## Docs

- `./docs-report.sh` builds the Doxygen API docs locally (`--site` for the full site).
- `./coverage-report.sh` builds with coverage and writes `coverage-html/index.html`.
