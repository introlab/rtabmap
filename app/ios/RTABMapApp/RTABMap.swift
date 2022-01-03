//
//  RTABMap.swift
//  RTABMapApp
//
//  Created by Mathieu Labbe on 2020-12-31.
//

import Foundation
import ARKit

class RTABMap {
    var native_rtabmap: UnsafeMutableRawPointer
    
    struct Observation {
        weak var observer: RTABMapObserver?
    }
    private var observations = [ObjectIdentifier : Observation]()
    
    func addObserver(_ observer: RTABMapObserver) {
        let id = ObjectIdentifier(observer)
        observations[id] = Observation(observer: observer)
    }

    func removeObserver(_ observer: RTABMapObserver) {
        let id = ObjectIdentifier(observer)
        observations.removeValue(forKey: id)
    }
    
    init() {
        native_rtabmap = UnsafeMutableRawPointer(mutating: createNativeApplication())
    }
    
    func setupCallbacksWithCPP()
    {
        setupCallbacksNative(native_rtabmap,
             UnsafeMutableRawPointer(Unmanaged.passUnretained(self).toOpaque()),
             //progressCallback
             {(observer, count, max) -> Void in
                         // Extract pointer to `self` from void pointer:
                let mySelf = Unmanaged<RTABMap>.fromOpaque(observer!).takeUnretainedValue()
                         // Call instance method:
                         //mySelf.TestMethod();
                for (id, observation) in mySelf.observations {
                    // If the observer is no longer in memory, we
                    // can clean up the observation for its ID
                    guard let observer = observation.observer else {
                        mySelf.observations.removeValue(forKey: id)
                        continue
                    }

                    observer.progressUpdated(mySelf, count: Int(count), max: Int(max))
                }
             },
             //initCallback
             {(observer, status, msg) -> Void in
                         // Extract pointer to `self` from void pointer:
                let mySelf = Unmanaged<RTABMap>.fromOpaque(observer!).takeUnretainedValue()
                         // Call instance method:
                         //mySelf.TestMethod();
                for (id, observation) in mySelf.observations {
                    // If the observer is no longer in memory, we
                    // can clean up the observation for its ID
                    guard let observer = observation.observer else {
                        mySelf.observations.removeValue(forKey: id)
                        continue
                    }

                    let str = String(cString: msg!)
                    observer.initEventReceived(mySelf, status: Int(status), msg: str)
                }
             },
             //statsUpdatedCallback
             {(observer, nodes, words, points, polygons, updateTime, loopClosureId, highestHypId, databaseMemoryUsed, inliers, matches, featuresExtracted, hypothesis, nodesDrawn, fps, rejected, rehearsalValue, optimizationMaxError, optimizationMaxErrorRatio, distanceTravelled, fastMovement, landmarkDetected, x, y, z, roll, pitch, yaw) -> Void in
                         // Extract pointer to `self` from void pointer:
                let mySelf = Unmanaged<RTABMap>.fromOpaque(observer!).takeUnretainedValue()
                         // Call instance method:
                         //mySelf.TestMethod();
                for (id, observation) in mySelf.observations {
                    // If the observer is no longer in memory, we
                    // can clean up the observation for its ID
                    guard let observer = observation.observer else {
                        mySelf.observations.removeValue(forKey: id)
                        continue
                    }

                    observer.statsUpdated(mySelf, nodes: Int(nodes), words: Int(words), points: Int(points), polygons: Int(polygons), updateTime: updateTime, loopClosureId: Int(loopClosureId), highestHypId: Int(highestHypId), databaseMemoryUsed: Int(databaseMemoryUsed), inliers: Int(inliers), matches: Int(matches), featuresExtracted: Int(featuresExtracted), hypothesis: hypothesis, nodesDrawn: Int(nodesDrawn), fps: fps, rejected: Int(rejected), rehearsalValue: rehearsalValue, optimizationMaxError: optimizationMaxError, optimizationMaxErrorRatio: optimizationMaxErrorRatio, distanceTravelled: distanceTravelled, fastMovement: Int(fastMovement), landmarkDetected: Int(landmarkDetected), x: x, y: y, z: z, roll: roll, pitch: pitch, yaw: yaw)
                }
             })
    }
    
    deinit {
        destroyNativeApplication(native_rtabmap)
    }
    
    func initGlContent() {
        initGlContentNative(native_rtabmap)
    }
    
    func setupGraphic(size: CGSize, orientation: UIInterfaceOrientation) {
        var rotation: Int32
        switch orientation.rawValue {
        case 4:
            rotation=2
        case 1:
            rotation=3
        case 2:
            rotation=1
        default:
            rotation=0
        }
        //NSLog("Orientation: ios %d android %d, view=%dx%d", orientation.rawValue, rotation, Int32(size.width), Int32(size.height))
        setScreenRotationNative(native_rtabmap, rotation)
        setupGraphicNative(native_rtabmap, Int32(size.width), Int32(size.height));
    }
    
    func openDatabase(databasePath:String, databaseInMemory:Bool, optimize:Bool, clearDatabase: Bool) -> Int {
        databasePath.utf8CString.withUnsafeBufferPointer { buffer -> Int in
            return Int(openDatabaseNative(native_rtabmap, buffer.baseAddress, databaseInMemory, optimize, clearDatabase))
        }
    }
    
    func save(databasePath:String) {
        databasePath.utf8CString.withUnsafeBufferPointer { buffer in
            saveNative(native_rtabmap, databasePath)
        }
    }
    
    func recover(from: String, to: String) -> Bool {
        from.utf8CString.withUnsafeBufferPointer { bufferFrom -> Bool in
            to.utf8CString.withUnsafeBufferPointer { bufferTo -> Bool in
                return recoverNative(native_rtabmap, bufferFrom.baseAddress, bufferTo.baseAddress)
            }
        }
    }
    
    func cancelProcessing() {
        cancelProcessingNative(native_rtabmap);
    }
    
    func postProcessing(approach: Int) -> Int {
        return Int(postProcessingNative(native_rtabmap, Int32(approach)))
    }
    
    func exportMesh(
        cloudVoxelSize: Float,
        regenerateCloud: Bool,
        meshing: Bool,
        textureSize: Int,
        textureCount: Int,
        normalK: Int,
        optimized: Bool,
        optimizedVoxelSize: Float,
        optimizedDepth: Int,
        optimizedMaxPolygons: Int,
        optimizedColorRadius: Float,
        optimizedCleanWhitePolygons: Bool,
        optimizedMinClusterSize: Int,
        optimizedMaxTextureDistance: Float,
        optimizedMinTextureClusterSize: Int,
        blockRendering: Bool) -> Bool
    {
       return exportMeshNative(native_rtabmap,
                                    cloudVoxelSize,
                                    regenerateCloud,
                                    meshing,
                                    Int32(textureSize),
                                    Int32(textureCount),
                                    Int32(normalK),
                                    optimized,
                                    optimizedVoxelSize,
                                    Int32(optimizedDepth),
                                    Int32(optimizedMaxPolygons),
                                    optimizedColorRadius,
                                    optimizedCleanWhitePolygons,
                                    Int32(optimizedMinClusterSize),
                                    optimizedMaxTextureDistance,
                                    Int32(optimizedMinTextureClusterSize),
                                    blockRendering)
    }
    
    func postExportation(visualize: Bool) -> Bool
    {
        return postExportationNative(native_rtabmap, visualize)
    }
    
    func writeExportedMesh(directory: String, name: String) -> Bool
    {
        directory.utf8CString.withUnsafeBufferPointer { bufferDir in
            name.utf8CString.withUnsafeBufferPointer { bufferName in
                return writeExportedMeshNative(native_rtabmap, bufferDir.baseAddress, bufferName.baseAddress)
            }
        }
    }
    
    func onTouchEvent(touch_count: Int, event: Int, x0: Float, y0: Float, x1: Float, y1: Float) {
        onTouchEventNative(native_rtabmap, Int32(touch_count), Int32(event), x0, y0, x1, y1)
    }
    
    func setPausedMapping(paused: Bool) {
        setPausedMappingNative(native_rtabmap, paused)
    }
    
    func render() -> Int {
        return Int(renderNative(native_rtabmap))
    }
    
    func startCamera() -> Bool {
        return startCameraNative(native_rtabmap)
    }
    
    func stopCamera() {
        stopCameraNative(native_rtabmap)
    }
    
    func setCamera(type: Int) {
        setCameraNative(native_rtabmap, Int32(type))
    }
    
    func postCameraPoseEvent(pose: simd_float4x4) {
        let rotation = GLKMatrix3(
            m: (pose[0,0], pose[0,1], pose[0,2],
            pose[1,0], pose[1,1], pose[1,2],
            pose[2,0], pose[2,1], pose[2,2]))
        let quat = GLKQuaternionMakeWithMatrix3(rotation)
        postCameraPoseEventNative(native_rtabmap, pose[3,0], pose[3,1], pose[3,2], quat.x, quat.y, quat.z, quat.w)
    }
    
    func notifyLost() {
        // a null transform will make rtabmap creating a new session
        postCameraPoseEventNative(native_rtabmap, 0,0,0,0,0,0,0)
    }
    
    func postOdometryEvent(frame: ARFrame, orientation: UIInterfaceOrientation, viewport: CGSize) {
        let pose = frame.camera.transform   // ViewMatrix
        let rotation = GLKMatrix3(
            m: (pose[0,0], pose[0,1], pose[0,2],
            pose[1,0], pose[1,1], pose[1,2],
            pose[2,0], pose[2,1], pose[2,2]))
        
        let quat = GLKQuaternionMakeWithMatrix3(rotation)
                
        postCameraPoseEventNative(native_rtabmap, pose[3,0], pose[3,1], pose[3,2], quat.x, quat.y, quat.z, quat.w)
                
        let confMap = frame.sceneDepth?.confidenceMap
        let depthMap = frame.sceneDepth?.depthMap
        let points = frame.rawFeaturePoints?.points
                
        if points != nil && (depthMap != nil || points!.count>0)
        {
            let v = frame.camera.viewMatrix(for: orientation)
            let p = frame.camera.projectionMatrix(for: orientation, viewportSize: viewport, zNear: 0.5, zFar: 50.0)
            
            let rotation = GLKMatrix3(
                m: (v[0,0], v[0,1], v[0,2],
                    v[1,0], v[1,1], v[1,2],
                    v[2,0], v[2,1], v[2,2]))
            
            let quatv = GLKQuaternionMakeWithMatrix3(rotation)
            
            let texX1 = (1-(2*frame.camera.intrinsics[0,0]/Float(frame.camera.imageResolution.width)) / p[0,0])/2
            let texY1 = (1-(2*frame.camera.intrinsics[1,1]/Float(frame.camera.imageResolution.height)) / p[1,1])/2
            
            let texX2 = (1-(2*frame.camera.intrinsics[0,0]/Float(frame.camera.imageResolution.width)) / p[1,1])/2
            let texY2 = (1-(2*frame.camera.intrinsics[1,1]/Float(frame.camera.imageResolution.height)) / p[0,0])/2
            
            //11 10 01 00 // portrait
            //01 11 00 10 // right
            //10 00 11 01 // left
            //00 01 10 11 // down
            var texCoord: [Float]
            switch orientation {
            case .portrait:
                texCoord = [1-texX2, 1-texY2, 1-texX2,texY2, texX2, 1-texY2, texX2, texY2]
            case .landscapeRight:
                texCoord = [texX1, 1-texY1, 1-texX1, 1-texY1, texX1, texY1, 1-texX1, texY1]
            case .landscapeLeft:
                texCoord = [1-texX1, texY1, texX1, texY1, 1-texX1, 1-texY1, texX1, 1-texY1]
            default: // down
                texCoord = [texX2, texY2, texX2, 1-texY2, 1-texX2, texY2, 1-texX2, 1-texY2]
            }
            
            frame.rawFeaturePoints?.points.withUnsafeBufferPointer { bufferPoints in
                
                CVPixelBufferLockBaseAddress(frame.capturedImage, CVPixelBufferLockFlags.readOnly)
                var depthDataPtr: UnsafeMutableRawPointer?
                var depthSize: Int32 = 0
                var depthWidth: Int32 = 0
                var depthHeight: Int32 = 0
                var depthFormat: Int32 = 0
                if depthMap != nil {
                    CVPixelBufferLockBaseAddress(depthMap!, CVPixelBufferLockFlags.readOnly)
                    depthDataPtr = CVPixelBufferGetBaseAddress(depthMap!)!
                    depthSize = Int32(CVPixelBufferGetDataSize(depthMap!))
                    depthWidth = Int32(CVPixelBufferGetWidth(depthMap!))
                    depthHeight = Int32(CVPixelBufferGetHeight(depthMap!))
                    depthFormat = Int32(CVPixelBufferGetPixelFormatType(depthMap!))
                }
                
                var confDataPtr: UnsafeMutableRawPointer?
                var confSize: Int32 = 0
                var confWidth: Int32 = 0
                var confHeight: Int32 = 0
                var confFormat: Int32 = 0
                if confMap != nil {
                    CVPixelBufferLockBaseAddress(confMap!, CVPixelBufferLockFlags.readOnly)
                    confDataPtr = CVPixelBufferGetBaseAddress(confMap!)!
                    confSize = Int32(CVPixelBufferGetDataSize(confMap!))
                    confWidth = Int32(CVPixelBufferGetWidth(confMap!))
                    confHeight = Int32(CVPixelBufferGetHeight(confMap!))
                    confFormat = Int32(CVPixelBufferGetPixelFormatType(confMap!))
                }
                
                if(frame.lightEstimate != nil) {
                    addEnvSensor(type: 4, value: Float(frame.lightEstimate!.ambientIntensity))
                }
                                
                postOdometryEventNative(native_rtabmap,
                                        pose[3,0], pose[3,1], pose[3,2], quat.x, quat.y, quat.z, quat.w,
                                        frame.camera.intrinsics[0,0], // fx
                                        frame.camera.intrinsics[1,1], // fy
                                        frame.camera.intrinsics[2,0], // cx
                                        frame.camera.intrinsics[2,1], // cy
                                        frame.timestamp,
                                        CVPixelBufferGetBaseAddressOfPlane(frame.capturedImage, 0),  // y plane pointer
                                        nil,                                                         // u plane pointer
                                        CVPixelBufferGetBaseAddressOfPlane(frame.capturedImage, 1),  // v plane pointer
                                        Int32(CVPixelBufferGetBytesPerRowOfPlane(frame.capturedImage, 0)) * Int32(CVPixelBufferGetHeightOfPlane(frame.capturedImage, 0)),  // yPlaneLen
                                        Int32(CVPixelBufferGetWidth(frame.capturedImage)),           // rgb width
                                        Int32(CVPixelBufferGetHeight(frame.capturedImage)),          // rgb height
                                        Int32(CVPixelBufferGetPixelFormatType(frame.capturedImage)), // rgb format
                                        depthDataPtr, // depth pointer
                                        depthSize,    // depth size
                                        depthWidth,   // depth width
                                        depthHeight,  // depth height
                                        depthFormat,  // depth format
                                        confDataPtr, // conf pointer
                                        confSize,    // conf size
                                        confWidth,   // conf width
                                        confHeight,  // conf height
                                        confFormat,  // conf format
                                        bufferPoints.baseAddress, Int32(frame.rawFeaturePoints!.points.count), 4,
                                        v[3,0], v[3,1], v[3,2], quatv.x, quatv.y, quatv.z, quatv.w,
                                        p[0,0], p[1,1], p[2,0], p[2,1], p[2,2], p[2,3], p[3,2],
                                        texCoord[0],texCoord[1],texCoord[2],texCoord[3],texCoord[4],texCoord[5],texCoord[6],texCoord[7])
                if depthMap != nil {
                    CVPixelBufferUnlockBaseAddress(depthMap!, CVPixelBufferLockFlags.readOnly)
                }
                if confMap != nil {
                    CVPixelBufferUnlockBaseAddress(confMap!, CVPixelBufferLockFlags.readOnly)
                }
                CVPixelBufferUnlockBaseAddress(frame.capturedImage, CVPixelBufferLockFlags.readOnly)
            }
        }
    }
        
    // Parameters
    func setOnlineBlending(enabled: Bool) {
        setOnlineBlendingNative(native_rtabmap, enabled)
    }
    func setMapCloudShown(shown: Bool) {
        setMapCloudShownNative(native_rtabmap, shown)
    }
    func setOdomCloudShown(shown: Bool) {
        setOdomCloudShownNative(native_rtabmap, shown)
    }
    func setMeshRendering(enabled: Bool, withTexture: Bool) {
        setMeshRenderingNative(native_rtabmap, enabled, withTexture)
    }
    func setPointSize(value: Float) {
        setPointSizeNative(native_rtabmap, value)
    }
    func setFOV(angle: Float) {
        setFOVNative(native_rtabmap, angle)
    }
    func setOrthoCropFactor(value: Float) {
        setOrthoCropFactorNative(native_rtabmap, value)
    }
    func setGridRotation(value: Float) {
        setGridRotationNative(native_rtabmap, value)
    }
    func setLighting(enabled: Bool) {
        setLightingNative(native_rtabmap, enabled)
    }
    func setBackfaceCulling(enabled: Bool) {
        setBackfaceCullingNative(native_rtabmap, enabled)
    }
    func setWireframe(enabled: Bool) {
        setWireframeNative(native_rtabmap, enabled)
    }
    func setLocalizationMode(enabled: Bool) {
        setLocalizationModeNative(native_rtabmap, enabled)
    }
    func setTrajectoryMode(enabled: Bool) {
        setTrajectoryModeNative(native_rtabmap, enabled)
    }
    func setGraphOptimization(enabled: Bool) {
        setGraphOptimizationNative(native_rtabmap, enabled)
    }
    func setNodesFiltering(enabled: Bool) {
        setNodesFilteringNative(native_rtabmap, enabled)
    }
    func setGraphVisible(visible: Bool) {
        setGraphVisibleNative(native_rtabmap, visible)
    }
    func setGridVisible(visible: Bool) {
        setGridVisibleNative(native_rtabmap, visible)
    }
    func setRawScanSaved(enabled: Bool) {
        setRawScanSavedNative(native_rtabmap, enabled)
    }
    func setFullResolution(enabled: Bool) {
        setFullResolutionNative(native_rtabmap, enabled)
    }
    func setSmoothing(enabled: Bool) {
        setSmoothingNative(native_rtabmap, enabled)
    }
    func setAppendMode(enabled: Bool) {
        setAppendModeNative(native_rtabmap, enabled)
    }
    func setMaxCloudDepth(value: Float) {
        setMaxCloudDepthNative(native_rtabmap, value)
    }
    func setMinCloudDepth(value: Float) {
        setMinCloudDepthNative(native_rtabmap, value)
    }
    func setCloudDensityLevel(value: Int) {
        setCloudDensityLevelNative(native_rtabmap, Int32(value))
    }
    func setMeshAngleTolerance(value: Float) {
        setMeshAngleToleranceNative(native_rtabmap, value)
    }
    func setMeshDecimationFactor(value: Float) {
        setMeshDecimationFactorNative(native_rtabmap, value)
    }
    func setMeshTriangleSize(value: Int) {
        setMeshTriangleSizeNative(native_rtabmap, Int32(value))
    }
    func setClusterRatio(value: Float) {
        setClusterRatioNative(native_rtabmap, value)
    }
    func setMaxGainRadius(value: Float) {
        setMaxGainRadiusNative(native_rtabmap, value)
    }
    func setRenderingTextureDecimation(value: Int) {
        setRenderingTextureDecimationNative(native_rtabmap, Int32(value))
    }
    func setBackgroundColor(gray: Float) {
        setBackgroundColorNative(native_rtabmap, gray)
    }
    func setDepthConfidence(value: Int) {
        setDepthConfidenceNative(native_rtabmap, Int32(value))
    }
    func setMappingParameter(key: String, value: String) {
        key.utf8CString.withUnsafeBufferPointer { bufferKey in
            value.utf8CString.withUnsafeBufferPointer { bufferValue in
                setMappingParameterNative(native_rtabmap, bufferKey.baseAddress, bufferValue.baseAddress)
            }
        }
    }
    func setGPS(location: CLLocation)
    {
        setGPSNative(native_rtabmap, location.timestamp.timeIntervalSince1970, location.coordinate.longitude, location.coordinate.latitude, location.altitude, location.horizontalAccuracy, location.course < 0.0 ? 0.0 : location.course)
    }
    func addEnvSensor(type: Int, value: Float)
    {
        addEnvSensorNative(native_rtabmap, Int32(type), value)
    }
    func setOrthoCropFactor(_ value: Float)
    {
        setOrthoCropFactorNative(native_rtabmap, value)
    }
    func setGridRotation(_ value: Float)
    {
        setGridRotationNative(native_rtabmap, value)
    }
}

func getPreviewImage(databasePath: String) -> UIImage?
{
    let imageOut = databasePath.utf8CString.withUnsafeBufferPointer { buffer -> UIImage? in
        var image = ImageNative()
        image = getPreviewImageNative(buffer.baseAddress)
        if let dataPtr = UnsafeRawPointer(image.data)
        {
            // copy data in swift
            let data = Data(bytes: dataPtr, count: Int(image.width*image.height*image.channels))
            // release memory
            releasePreviewImageNative(image)
            // Create bitmap image
            let bitmap = CIImage(bitmapData: data, bytesPerRow: Int(image.width*image.channels), size: CGSize(width: Int(image.width), height: Int(image.height)), format: CIFormat.BGRA8, colorSpace: nil)
            return UIImage(ciImage: bitmap)
        }
        return UIImage(named: "RTAB-Map1024")
    }
    return imageOut
}

protocol RTABMapObserver: class {
    func progressUpdated(_ rtabmap: RTABMap, count: Int, max: Int)
    func initEventReceived(_ rtabmap: RTABMap, status: Int, msg: String)
    func statsUpdated(_ rtabmap: RTABMap,
                      nodes: Int,
                      words: Int,
                      points: Int,
                      polygons: Int,
                      updateTime: Float,
                      loopClosureId: Int,
                      highestHypId: Int,
                      databaseMemoryUsed: Int,
                      inliers: Int,
                      matches: Int,
                      featuresExtracted: Int,
                      hypothesis: Float,
                      nodesDrawn: Int,
                      fps: Float,
                      rejected: Int,
                      rehearsalValue: Float,
                      optimizationMaxError: Float,
                      optimizationMaxErrorRatio: Float,
                      distanceTravelled: Float,
                      fastMovement: Int,
                      landmarkDetected: Int,
                      x: Float,
                      y: Float,
                      z: Float,
                      roll: Float,
                      pitch: Float,
                      yaw: Float)
}

extension String {

  func toPointer() -> UnsafePointer<UInt8>? {
    guard let data = self.data(using: String.Encoding.utf8) else { return nil }

    let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: data.count)
    let stream = OutputStream(toBuffer: buffer, capacity: data.count)

    stream.open()
    data.withUnsafeBytes({ (p: UnsafePointer<UInt8>) -> Void in
      stream.write(p, maxLength: data.count)
    })

    stream.close()

    return UnsafePointer<UInt8>(buffer)
  }
}

