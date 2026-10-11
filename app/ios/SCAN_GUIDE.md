# Scanning rooms and outdoor objects on iOS

Use an iPhone or iPad with LiDAR. The App Store release is available at
<https://apps.apple.com/app/rtab-map-3d-lidar-scanner/id1564774365>.
Changes on a development branch are not automatically included in that release.

## A first room

1. Start a new mapping session. Leave **LiDAR Mode** enabled in the mapping
   settings and use normal mapping, rather than Data Recording Mode, to see the
   scan as it grows.
2. Begin at a distinctive corner with furniture or other visual detail. Wait for
   camera initialization, then press Record.
3. Move slowly around the room. Overlap neighboring views, include the floor and
   wall junctions, and look at missing surfaces from another angle. Avoid spending
   the entire scan pointing at a featureless wall, mirror or window.
4. Return to the starting area before stopping. Recognizing a previously seen
   area helps RTAB-Map align the scan. A tracking recovery message alone does not
   mean the map has aligned successfully.
5. Stop and save the database. Keep this `.db` file: it contains the captured data
   for later processing. Assemble an optimized textured mesh and export OBJ for
   a model with images, or export a point cloud for further processing.
6. Inspect the result before leaving the room. Check wall alignment and gaps, and
   compare several known distances with a tape measure. Mesh resolution is not
   a guarantee of measurement accuracy.

## An outdoor gate or similar object

- Keep the gate still for the whole scan. Prefer even light; strong sunlight,
  reflective metal and dark surfaces can reduce depth quality.
- Move slowly along the gate with overlapping views. Include posts, paving or
  other stationary surroundings so the camera has recognizable features.
- Capture both sides if accessible, following a continuous route around an end
  with overlap. Returning to the initial view can help close the loop.
- Inspect narrow bars, holes and edges. A reconstructed mesh can fill real gaps
  or miss thin parts; also save a point cloud to check what was actually captured.
- Save a short test first and inspect it before attempting a longer scan. No
  capture mode can guarantee complete geometry on every material or in all light.

The **Scanning Tips** menu provides a short version of these instructions in the
development app.

## Tracking recovery

During mapping and Data Recording Mode, a loss of tracking after the first usable
pose now takes priority over ordinary toast messages. The message remains visible
while tracking is unavailable. Once tracking resumes, an eight-second reminder
asks the user to revisit previously scanned surroundings. Normal initialization,
camera preview and model viewing do not produce lost-scan warnings.

This is guidance for preserving overlap, not a fix for ARKit tracking loss or an
automatic repair of a disconnected map. Data Recording Mode does not detect loop
closures online; its recordings need post-processing.

## Build and test the contribution

Follow the project's [iOS build instructions](https://github.com/introlab/rtabmap/wiki/Installation#ios)
on a Mac with Xcode. Open `RTABMapApp.xcodeproj`, select your signing team and a
unique bundle identifier, connect a LiDAR iPhone, and build the app. Keep exported
scan backups before switching development builds or bundle identifiers.

The workflow produces an **unsigned** IPA for build verification. This is not
directly installable on an iPhone and is not a TestFlight release.

The state transition tests need only Swift and can run independently of the iOS
SDK, camera and native C++ dependencies:

```sh
bash app/ios/Tests/run-scan-tracking-tests.sh
```

### Device validation still required

Test both normal mapping and Data Recording Mode on a LiDAR device:

1. Initialize and begin recording: no false recovery warning.
2. Cause an actual ARKit tracking loss during capture: guidance replaces an
   existing toast and remains visible longer than its old dismissal timer.
3. Recover tracking: the overlap reminder lasts eight seconds, then clears.
4. Lose tracking again during that reminder: the loss message takes priority.
5. Stop, save, resume and start a new scan: no stale warning or reminder.
6. Interrupt the AR session; check recovery, background/foreground behavior,
   portrait/landscape readability and VoiceOver announcements.
7. Scan a room and an outdoor object, save/reopen their databases, and inspect
   exported geometry and textures. Compare with the unmodified build.

Covering the lens may only produce an `insufficientFeatures` state rather than
actual tracking loss. That state retains the existing warning and is not treated
as a new map. This change does not claim to reproduce or solve the underlying
long-recording failure reported in issue #1682.
