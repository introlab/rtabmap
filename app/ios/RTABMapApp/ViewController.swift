//
//  ViewController.swift
//  GLKittutorial
//
//  Created by Mathieu Labbe on 2020-12-28.
//

import GLKit
import ARKit
import Zip
import StoreKit

extension Array {
    func size() -> Int {
        return MemoryLayout<Element>.stride * self.count
    }
}

class ViewController: GLKViewController, ARSessionDelegate, RTABMapObserver, UIPickerViewDataSource, UIPickerViewDelegate, CLLocationManagerDelegate {
    
    private let session = ARSession()
    private var locationManager: CLLocationManager?
    private var mLastKnownLocation: CLLocation?
    private var mLastLightEstimate: CGFloat?
    
    private var context: EAGLContext?
    private var rtabmap: RTABMap?
    
    private var databases = [URL]()
    private var currentDatabaseIndex: Int = 0
    private var openedDatabasePath: URL?
    
    private var progressDialog: UIAlertController?
    var progressView : UIProgressView?
    
    var maxPolygonsPickerView: UIPickerView!
    var maxPolygonsPickerData: [Int]!
    
    private var mTotalLoopClosures: Int = 0
    private var mMapNodes: Int = 0
    private var mTimeThr: Int = 0
    private var mMaxFeatures: Int = 0
    private var mLoopThr = 0.11
    
    private var mReviewRequested = false
    
    // UI states
    private enum State {
        case STATE_WELCOME,    // Camera/Motion off - showing only buttons open and start new scan
        STATE_CAMERA,          // Camera/Motion on - not mapping
        STATE_MAPPING,         // Camera/Motion on - mapping
        STATE_IDLE,            // Camera/Motion off
        STATE_PROCESSING,      // Camera/Motion off - post processing
        STATE_VISUALIZING,     // Camera/Motion off - Showing optimized mesh
        STATE_VISUALIZING_CAMERA,     // Camera/Motion on  - Showing optimized mesh
        STATE_VISUALIZING_WHILE_LOADING // Camera/Motion off - Loading data while showing optimized mesh
    }
    private var mState: State = State.STATE_WELCOME;
    private func getStateString(state: State) -> String {
        switch state {
        case .STATE_WELCOME:
            return "Welcome"
        case .STATE_CAMERA:
            return "Camera Preview"
        case .STATE_MAPPING:
            return "Mapping"
        case .STATE_PROCESSING:
            return "Processing"
        case .STATE_VISUALIZING:
            return "Visualizing"
        case .STATE_VISUALIZING_CAMERA:
            return "Visualizing with Camera"
        case .STATE_VISUALIZING_WHILE_LOADING:
            return "Visualizing while Loading"
        default: // IDLE
            return "Idle"
        }
    }
    
    private var depthSupported: Bool = false
    
    private var viewMode: Int = 2 // 0=Cloud, 1=Mesh, 2=Textured Mesh
    private var cameraMode: Int = 1
    
    private var statusShown: Bool = true
    private var debugShown: Bool = false
    private var mapShown: Bool = true
    private var odomShown: Bool = true
    private var graphShown: Bool = true
    private var gridShown: Bool = true
    private var optimizedGraphShown: Bool = true
    private var wireframeShown: Bool = false
    private var backfaceShown: Bool = false
    private var lightingShown: Bool = false
    private var mHudVisible: Bool = true
    private var mLastTimeHudShown: DispatchTime = .now()
    private var mMenuOpened: Bool = false
    
    @IBOutlet weak var stopButton: UIButton!
    @IBOutlet weak var recordButton: UIButton!
    @IBOutlet weak var menuButton: UIButton!
    @IBOutlet weak var viewButton: UIButton!
    @IBOutlet weak var newScanButtonLarge: UIButton!
    @IBOutlet weak var libraryButton: UIButton!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var closeVisualizationButton: UIButton!
    @IBOutlet weak var stopCameraButton: UIButton!
    @IBOutlet weak var exportOBJPLYButton: UIButton!
    @IBOutlet weak var orthoDistanceSlider: UISlider!{
        didSet{
            orthoDistanceSlider.transform = CGAffineTransform(rotationAngle: CGFloat(-Double.pi/2))
        }
    }
    @IBOutlet weak var orthoGridSlider: UISlider!
    @IBOutlet weak var toastLabel: UILabel!
    
    let RTABMAP_TMP_DB = "rtabmap.tmp.db"
    let RTABMAP_RECOVERY_DB = "rtabmap.tmp.recovery.db"
    let RTABMAP_EXPORT_DIR = "Export"

    func getDocumentDirectory() -> URL {
        return FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
    }
    
    func getTmpDirectory() -> URL {
       return URL(fileURLWithPath: NSTemporaryDirectory())
    }
    
    @objc func defaultsChanged(){
        updateDisplayFromDefaults()
    }
    
    func showToast(message : String, seconds: Double){
        if(!self.toastLabel.isHidden)
        {
            return;
        }
        self.toastLabel.text = message
        self.toastLabel.isHidden = false
        DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + seconds) {
            self.toastLabel.isHidden = true
        }
    }
    
    func resetNoTouchTimer(_ showHud: Bool = false) {
        if(showHud)
        {
            print("Show HUD")
            mMenuOpened = false
            mHudVisible = true
            setNeedsStatusBarAppearanceUpdate()
            updateState(state: self.mState)
            
            mLastTimeHudShown = DispatchTime.now()
            DispatchQueue.main.asyncAfter(deadline: DispatchTime.now() + 5) {
                if(DispatchTime.now() <= self.mLastTimeHudShown + 4.9) {
                    return
                }
                if(self.mState != .STATE_WELCOME && self.mState != .STATE_CAMERA && self.presentedViewController as? UIAlertController == nil && !self.mMenuOpened)
                {
                    print("Hide HUD")
                    self.mHudVisible = false
                    self.setNeedsStatusBarAppearanceUpdate()
                    self.updateState(state: self.mState)
                }
            }
        }
        else if(mState != .STATE_WELCOME && mState != .STATE_CAMERA && presentedViewController as? UIAlertController == nil && !mMenuOpened)
        {
            print("Hide HUD")
            self.mHudVisible = false
            self.setNeedsStatusBarAppearanceUpdate()
            self.updateState(state: self.mState)
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        
        self.toastLabel.isHidden = true
        session.delegate = self
        
        depthSupported = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        
        rtabmap = RTABMap()
        rtabmap?.setupCallbacksWithCPP()
        
        context = EAGLContext(api: .openGLES2)
        EAGLContext.setCurrent(context)
        
        if let view = self.view as? GLKView, let context = context {
            view.context = context
            delegate = self
            rtabmap?.initGlContent()
        }
        
        menuButton.showsMenuAsPrimaryAction = true
        viewButton.showsMenuAsPrimaryAction = true
        statusLabel.numberOfLines = 0
        statusLabel.text = ""
        
        updateDatabases()
        
        let doubleTap = UITapGestureRecognizer(target: self, action: #selector(doubleTapped(_:)))
        doubleTap.numberOfTapsRequired = 2
        view.addGestureRecognizer(doubleTap)
        let singleTap = UITapGestureRecognizer(target: self, action: #selector(singleTapped(_:)))
        singleTap.numberOfTapsRequired = 1
        view.addGestureRecognizer(singleTap)
        
        let notificationCenter = NotificationCenter.default
        notificationCenter.addObserver(self, selector: #selector(appMovedToBackground), name: UIApplication.willResignActiveNotification, object: nil)
        notificationCenter.addObserver(self, selector: #selector(appMovedToForeground), name: UIApplication.willEnterForegroundNotification, object: nil)
        notificationCenter.addObserver(self, selector: #selector(defaultsChanged), name: UserDefaults.didChangeNotification, object: nil)
        
        rtabmap!.addObserver(self)
        
        registerSettingsBundle()
        updateDisplayFromDefaults()
        
        maxPolygonsPickerView = UIPickerView(frame: CGRect(x: 10, y: 50, width: 250, height: 150))
        maxPolygonsPickerView.delegate = self
        maxPolygonsPickerView.dataSource = self

        // This is where you can set your min/max values
        let minNum = 0
        let maxNum = 9
        maxPolygonsPickerData = Array(stride(from: minNum, to: maxNum + 1, by: 1))
        
        orthoDistanceSlider.setValue(80, animated: false)
        orthoGridSlider.setValue(90, animated: false)
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
            self.updateState(state: self.mState)
        }
    }
    
    func progressUpdated(_ rtabmap: RTABMap, count: Int, max: Int) {
        DispatchQueue.main.async {
            self.progressView?.setProgress(Float(count)/Float(max), animated: true)
        }
    }
    func initEventReceived(_ rtabmap: RTABMap, status: Int, msg: String) {
        DispatchQueue.main.async {
            var optimizedMeshDetected = 0

            if(msg == "Loading optimized cloud...done!")
            {
                optimizedMeshDetected = 1;
            }
            else if(msg == "Loading optimized mesh...done!")
            {
                optimizedMeshDetected = 2;
            }
            else if(msg == "Loading optimized texture mesh...done!")
            {
                optimizedMeshDetected = 3;
            }
            if(optimizedMeshDetected > 0)
            {
                if(optimizedMeshDetected==1)
                {
                    self.setMeshRendering(viewMode: 0)
                }
                else if(optimizedMeshDetected==2)
                {
                    self.setMeshRendering(viewMode: 1)
                }
                else // isOBJ
                {
                    self.setMeshRendering(viewMode: 2)
                }

                self.updateState(state: .STATE_VISUALIZING_WHILE_LOADING);
                self.setGLCamera(type: 2);
                
                self.dismiss(animated: true)
                self.showToast(message: "Optimized mesh detected in the database, it is shown while the database is loading...", seconds: 3)
            }

            let usedMem = self.getMemoryUsage()
            self.statusLabel.text =
                "Status: " + (status == 1 && msg.isEmpty ? self.mState == State.STATE_CAMERA ? "Camera Preview" : "Idle" : msg) + "\n" +
                "Memory Usage: \(usedMem) MB"
        }
    }
        
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
    {
        let usedMem = self.getMemoryUsage()
        
        if(loopClosureId > 0)
        {
            mTotalLoopClosures += 1;
        }
        let previousNodes = mMapNodes
        mMapNodes = nodes;
        
        let formattedDate = Date().getFormattedDate(format: "HH:mm:ss.SSS")
        
        DispatchQueue.main.async {
            
            if(self.mMapNodes>0 && previousNodes==0 && self.mState != .STATE_MAPPING)
            {
                self.updateState(state: self.mState) // refesh menus and actions
            }
            
            self.statusLabel.text = ""
            if self.statusShown {
                self.statusLabel.text =
                    self.statusLabel.text! +
                    "Status: \(self.getStateString(state: self.mState))\n" +
                    "Memory Usage : \(usedMem) MB"
            }
            if self.debugShown {
                self.statusLabel.text =
                    self.statusLabel.text! + "\n"
                var gpsString = "\n"
                if(UserDefaults.standard.bool(forKey: "SaveGPS"))
                {
                    if(self.mLastKnownLocation != nil)
                    {
                        let secondsOld = (Date().timeIntervalSince1970 - self.mLastKnownLocation!.timestamp.timeIntervalSince1970)
                        var bearing = 0.0
                        if(self.mLastKnownLocation!.course > 0.0) {
                            bearing = self.mLastKnownLocation!.course
                            
                        }
                        gpsString = String(format: "GPS: %.2f %.2f %.2fm %ddeg %.0fm [%d sec old]\n",
                                           self.mLastKnownLocation!.coordinate.longitude,
                                           self.mLastKnownLocation!.coordinate.latitude,
                                           self.mLastKnownLocation!.altitude,
                                           Int(bearing),
                                           self.mLastKnownLocation!.horizontalAccuracy,
                                           Int(secondsOld));
                    }
                    else
                    {
                        gpsString = "GPS: [not yet available]\n";
                    }
                }
                var lightString = "\n"
                if(self.mLastLightEstimate != nil)
                {
                    lightString = String("Light (lm): \(Int(self.mLastLightEstimate!))\n")
                }
                
                self.statusLabel.text =
                    self.statusLabel.text! +
                    gpsString + //gps
                    lightString + //env sensors
                    "Time: \(formattedDate)\n" +
                    "Nodes (WM): \(nodes) (\(nodesDrawn) shown)\n" +
                    "Words: \(words)\n" +
                    "Database (MB): \(databaseMemoryUsed)\n" +
                    "Number of points: \(points)\n" +
                    "Polygons: \(polygons)\n" +
                    "Update time (ms): \(Int(updateTime)) / \(self.mTimeThr==0 ? "No Limit" : String(self.mTimeThr))\n" +
                    "Features: \(featuresExtracted) / \(self.mMaxFeatures==0 ? "No Limit" : (self.mMaxFeatures == -1 ? "Disabled" : String(self.mMaxFeatures)))\n" +
                    "Rehearsal (%): \(Int(rehearsalValue*100))\n" +
                    "Loop closures: \(self.mTotalLoopClosures)\n" +
                    "Inliers: \(inliers)\n" +
                    "Hypothesis (%): \(Int(hypothesis*100)) / \(Int(self.mLoopThr*100)) (\(loopClosureId>0 ? loopClosureId : highestHypId))\n" +
                    String(format: "FPS (rendering): %.1f Hz\n", fps) +
                    String(format: "Travelled distance: %.2f m\n", distanceTravelled) +
                    String(format: "Pose (x,y,z): %.2f %.2f %.2f", x, y, z)
            }
            if(self.mState == .STATE_MAPPING || self.mState == .STATE_VISUALIZING_CAMERA)
            {
                if(loopClosureId > 0) {
                    if(self.mState == .STATE_VISUALIZING_CAMERA) {
                        self.showToast(message: "Localized!", seconds: 1);
                    }
                    else {
                        self.showToast(message: "Loop closure detected!", seconds: 1);
                    }
                }
                else if(rejected > 0)
                {
                    if(inliers >= UserDefaults.standard.integer(forKey: "MinInliers"))
                    {
                        if(optimizationMaxError > 0.0)
                        {
                            self.showToast(message: String(format: "Loop closure rejected, too high graph optimization error (%.3fm: ratio=%.3f < factor=%.1fx).", optimizationMaxError, optimizationMaxErrorRatio, UserDefaults.standard.float(forKey: "MaxOptimizationError")), seconds: 1);
                        }
                        else
                        {
                            self.showToast(message: String(format: "Loop closure rejected, graph optimization failed! You may try a different Graph Optimizer (see Mapping options)."), seconds: 1);
                        }
                    }
                    else
                    {
                        self.showToast(message: String(format: "Loop closure rejected, not enough inliers (%d/%d < %d).", inliers, matches, UserDefaults.standard.integer(forKey: "MinInliers")), seconds: 1);
                    }
                }
                else if(landmarkDetected > 0) {
                    self.showToast(message: "Landmark \(landmarkDetected) detected!", seconds: 1);
                }
            }
        }
    }
    
    func getMemoryUsage() -> UInt64 {
        var taskInfo = mach_task_basic_info()
        var count = mach_msg_type_number_t(MemoryLayout<mach_task_basic_info>.size)/4
        let kerr: kern_return_t = withUnsafeMutablePointer(to: &taskInfo) {
            $0.withMemoryRebound(to: integer_t.self, capacity: 1) {
                task_info(mach_task_self_, task_flavor_t(MACH_TASK_BASIC_INFO), $0, &count)
            }
        }

        if kerr == KERN_SUCCESS {
            return taskInfo.resident_size / (1024*1024)
        }
        else {
            print("Error with task_info(): " +
                (String(cString: mach_error_string(kerr), encoding: String.Encoding.ascii) ?? "unknown error"))
            return 0
        }
    }
    
    @objc func appMovedToBackground() {
        if(mState == .STATE_VISUALIZING_CAMERA || mState == .STATE_MAPPING || mState == .STATE_CAMERA)
        {
            stopMapping(ignoreSaving: true)
        }
    }
    
    @objc func appMovedToForeground() {
        updateDisplayFromDefaults()
        
        if(mMapNodes > 0 && self.openedDatabasePath == nil)
        {
            let msg = "RTAB-Map has been pushed to background while mapping. Do you want to save the map now?"
            let alert = UIAlertController(title: "Mapping Stopped!", message: msg, preferredStyle: .alert)
            let alertActionNo = UIAlertAction(title: "Ignore", style: .cancel) {
                (UIAlertAction) -> Void in
            }
            alert.addAction(alertActionNo)
            let alertActionYes = UIAlertAction(title: "Yes", style: .default) {
                (UIAlertAction) -> Void in
                self.save()
            }
            alert.addAction(alertActionYes)
            self.present(alert, animated: true, completion: nil)
        }
    }
    
    func setMeshRendering(viewMode: Int)
    {
        switch viewMode {
        case 0:
            self.rtabmap?.setMeshRendering(enabled: false, withTexture: false)
        case 1:
            self.rtabmap?.setMeshRendering(enabled: true, withTexture: false)
        default:
            self.rtabmap?.setMeshRendering(enabled: true, withTexture: true)
        }
        self.viewMode = viewMode
        updateState(state: mState)
    }
    
    func setGLCamera(type: Int)
    {
        cameraMode = type
        rtabmap!.setCamera(type: type);
    }
    
    func startCamera()
    {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
            case .authorized: // The user has previously granted access to the camera.
                print("Start Camera")
                rtabmap!.startCamera()
                let configuration = ARWorldTrackingConfiguration()
                var message = ""
                if(!UserDefaults.standard.bool(forKey: "LidarMode"))
                {
                    message = "LiDAR is disabled (Settings->Mapping->LiDAR Mode = OFF), only tracked features will be mapped."
                    self.setMeshRendering(viewMode: 0)
                }
                else if !depthSupported
                {
                    message = "The device does not have a LiDAR, only tracked features will be mapped. A LiDAR is required for accurate 3D reconstruction."
                    self.setMeshRendering(viewMode: 0)
                }
                else
                {
                    configuration.frameSemantics = .sceneDepth
                }
                
                session.run(configuration, options: [.resetSceneReconstruction, .resetTracking, .removeExistingAnchors])
                
                switch mState {
                case .STATE_VISUALIZING:
                    updateState(state: .STATE_VISUALIZING_CAMERA)
                default:
                    locationManager?.startUpdatingLocation()
                    updateState(state: .STATE_CAMERA)
                }
                
                if(!message.isEmpty)
                {
                    let alertController = UIAlertController(title: "Start Camera", message: message, preferredStyle: .alert)
                    let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
                    }
                    alertController.addAction(okAction)
                    present(alertController, animated: true)
                }
            
            case .notDetermined: // The user has not yet been asked for camera access.
                AVCaptureDevice.requestAccess(for: .video) { granted in
                    if granted {
                        DispatchQueue.main.async {
                            self.startCamera()
                        }
                    }
                }
            
        default:
            let alertController = UIAlertController(title: "Camera Disabled", message: "Camera permission is required to start the camera. You can enable it in Settings.", preferredStyle: .alert)

            let settingsAction = UIAlertAction(title: "Settings", style: .default) { (action) in
                guard let settingsUrl = URL(string: UIApplication.openSettingsURLString) else {
                    return
                }
                if UIApplication.shared.canOpenURL(settingsUrl) {
                    UIApplication.shared.open(settingsUrl, completionHandler: { (success) in
                        print("Settings opened: \(success)") // Prints true
                    })
                }
            }
            alertController.addAction(settingsAction)
            
            let okAction = UIAlertAction(title: "Ignore", style: .default) { (action) in
            }
            alertController.addAction(okAction)
            
            present(alertController, animated: true)
        }
    }
    
    private func updateState(state: State)
    {
        print("State: \(state)")
        
        if(mState != state)
        {
            mState = state;
            resetNoTouchTimer(true)
            return
        }
        
        mState = state;

        var actionNewScanEnabled: Bool
        var actionSaveEnabled: Bool
        var actionResumeEnabled: Bool
        var actionExportEnabled: Bool
        var actionOptimizeEnabled: Bool
        var actionSettingsEnabled: Bool
        
        switch mState {
        case .STATE_CAMERA:
            libraryButton.isEnabled = false
            libraryButton.isHidden = false
            menuButton.isHidden = false
            viewButton.isHidden = false
            newScanButtonLarge.isHidden = true // WELCOME button
            recordButton.isHidden = false
            stopButton.isHidden = true
            closeVisualizationButton.isHidden = true
            stopCameraButton.isHidden = false
            exportOBJPLYButton.isHidden = true
            orthoDistanceSlider.isHidden = cameraMode != 3
            orthoGridSlider.isHidden = cameraMode != 3
            actionNewScanEnabled = true
            actionSaveEnabled = false
            actionResumeEnabled = false
            actionExportEnabled = false
            actionOptimizeEnabled = false
            actionSettingsEnabled = false
        case .STATE_MAPPING:
            libraryButton.isEnabled = false
            libraryButton.isHidden = !mHudVisible
            menuButton.isHidden = !mHudVisible
            viewButton.isHidden = !mHudVisible
            newScanButtonLarge.isHidden = true // WELCOME button
            recordButton.isHidden = true
            stopButton.isHidden = false
            closeVisualizationButton.isHidden = true
            stopCameraButton.isHidden = true
            exportOBJPLYButton.isHidden = true
            orthoDistanceSlider.isHidden = cameraMode != 3 || !mHudVisible
            orthoGridSlider.isHidden = cameraMode != 3 || !mHudVisible
            actionNewScanEnabled = true
            actionSaveEnabled = false
            actionResumeEnabled = false
            actionExportEnabled = false
            actionOptimizeEnabled = false
            actionSettingsEnabled = false
        case .STATE_PROCESSING,
             .STATE_VISUALIZING_WHILE_LOADING,
             .STATE_VISUALIZING_CAMERA:
            libraryButton.isEnabled = false
            libraryButton.isHidden = !mHudVisible
            menuButton.isHidden = !mHudVisible
            viewButton.isHidden = !mHudVisible
            newScanButtonLarge.isHidden = true // WELCOME button
            recordButton.isHidden = true
            stopButton.isHidden = true
            closeVisualizationButton.isHidden = true
            stopCameraButton.isHidden = mState != .STATE_VISUALIZING_CAMERA
            exportOBJPLYButton.isHidden = true
            orthoDistanceSlider.isHidden = cameraMode != 3 || mState != .STATE_VISUALIZING_WHILE_LOADING
            orthoGridSlider.isHidden = cameraMode != 3 || mState != .STATE_VISUALIZING_WHILE_LOADING
            actionNewScanEnabled = false
            actionSaveEnabled = false
            actionResumeEnabled = false
            actionExportEnabled = false
            actionOptimizeEnabled = false
            actionSettingsEnabled = false
        case .STATE_VISUALIZING:
            libraryButton.isEnabled = !databases.isEmpty
            libraryButton.isHidden = !mHudVisible
            menuButton.isHidden = !mHudVisible
            viewButton.isHidden = !mHudVisible
            newScanButtonLarge.isHidden = true // WELCOME button
            recordButton.isHidden = true
            stopButton.isHidden = true
            closeVisualizationButton.isHidden = !mHudVisible
            stopCameraButton.isHidden = true
            exportOBJPLYButton.isHidden = !mHudVisible
            orthoDistanceSlider.isHidden = cameraMode != 3 || !mHudVisible
            orthoGridSlider.isHidden = cameraMode != 3 || !mHudVisible
            actionNewScanEnabled = true
            actionSaveEnabled = mMapNodes>0
            actionResumeEnabled = mMapNodes>0
            actionExportEnabled = mMapNodes>0
            actionOptimizeEnabled = mMapNodes>0
            actionSettingsEnabled = true
        default: // IDLE // WELCOME
            libraryButton.isEnabled = !databases.isEmpty
            libraryButton.isHidden = mState != .STATE_WELCOME && !mHudVisible
            menuButton.isHidden = mState != .STATE_WELCOME && !mHudVisible
            viewButton.isHidden = mState != .STATE_WELCOME && !mHudVisible
            newScanButtonLarge.isHidden = mState != .STATE_WELCOME
            recordButton.isHidden = true
            stopButton.isHidden = true
            closeVisualizationButton.isHidden = true
            stopCameraButton.isHidden = true
            exportOBJPLYButton.isHidden = true
            orthoDistanceSlider.isHidden = cameraMode != 3 || !mHudVisible
            orthoGridSlider.isHidden = cameraMode != 3 || !mHudVisible
            actionNewScanEnabled = true
            actionSaveEnabled = mState != .STATE_WELCOME && mMapNodes>0
            actionResumeEnabled = mState != .STATE_WELCOME && mMapNodes>0
            actionExportEnabled = mState != .STATE_WELCOME && mMapNodes>0
            actionOptimizeEnabled = mState != .STATE_WELCOME && mMapNodes>0
            actionSettingsEnabled = true
        }

        let view = self.view as? GLKView
        if(mState != .STATE_MAPPING && mState != .STATE_CAMERA && mState != .STATE_VISUALIZING_CAMERA)
        {
            self.isPaused = true
            view?.enableSetNeedsDisplay = true
            self.view.setNeedsDisplay()
            print("enableSetNeedsDisplay")
        }
        else
        {
            view?.enableSetNeedsDisplay = false
            self.isPaused = false
            print("diaableSetNeedsDisplay")
        }
        
        if !self.isPaused {
            self.view.setNeedsDisplay()
        }
        
        // Update menus based on current state
        
        // PointCloud menu
        let pointCloudMenu = UIMenu(title: "Point cloud...", children: [
            UIAction(title: "Current Density", handler: { _ in
                self.export(isOBJ: false, meshing: false, regenerateCloud: false, optimized: false, optimizedMaxPolygons: 0, previousState: self.mState)
            }),
            UIAction(title: "Max Density", handler: { _ in
                self.export(isOBJ: false, meshing: false, regenerateCloud: true, optimized: false, optimizedMaxPolygons: 0, previousState: self.mState)
            })
        ])
        // Optimized Mesh menu
        let optimizedMeshMenu = UIMenu(title: "Optimized mesh...", children: [
            UIAction(title: "Colored Mesh", handler: { _ in
                self.exportMesh(isOBJ: false)
            }),
            UIAction(title: "Textured Mesh", handler: { _ in
                self.exportMesh(isOBJ: true)
            })
        ])
        
        // Export menu
        let exportMenu = UIMenu(title: "Assemble...", children: [pointCloudMenu, optimizedMeshMenu])
        
        // Optimized Mesh menu
        let optimizeAdvancedMenu = UIMenu(title: "Advanced...", children: [
            UIAction(title: "Global Graph Optimization", handler: { _ in
                self.optimization(approach: 0)
            }),
            UIAction(title: "Detect More Loop Closures", handler: { _ in
                self.optimization(approach: 2)
            }),
            UIAction(title: "Adjust Colors (Fast)", handler: { _ in
                self.optimization(approach: 5)
            }),
            UIAction(title: "Adjust Colors (Full)", handler: { _ in
                self.optimization(approach: 6)
            }),
            UIAction(title: "Mesh Smoothing", handler: { _ in
                self.optimization(approach: 7)
            }),
            UIAction(title: "Bundle Adjustment", handler: { _ in
                self.optimization(approach: 1)
            }),
            UIAction(title: "Noise Filtering", handler: { _ in
                self.optimization(approach: 4)
            })
        ])
        
        // Optimize menu
        let optimizeMenu = UIMenu(title: "Optimize...", children: [
            UIAction(title: "Standard Optimization", handler: { _ in
                self.optimization(approach: -1)
            }),
            optimizeAdvancedMenu])
                
        var fileMenuChildren: [UIMenuElement] = []
        fileMenuChildren.append(UIAction(title: "New Scan", image: UIImage(systemName: "plus.app"), attributes: actionNewScanEnabled ? [] : .disabled, state: .off, handler: { _ in
            self.newScan()
        }))
        if(actionOptimizeEnabled) {
            fileMenuChildren.append(optimizeMenu)
        }
        else {
            fileMenuChildren.append(UIAction(title: "Optimize...", attributes: .disabled, state: .off, handler: { _ in
            }))
        }
        if(actionExportEnabled) {
            fileMenuChildren.append(exportMenu)
        }
        else {
            fileMenuChildren.append(UIAction(title: "Assemble...", attributes: .disabled, state: .off, handler: { _ in
            }))
        }
        fileMenuChildren.append(UIAction(title: "Save", image: UIImage(systemName: "square.and.arrow.down"), attributes: actionSaveEnabled ? [] : .disabled, state: .off, handler: { _ in
            self.save()
        }))
        fileMenuChildren.append(UIAction(title: "Append Scan", image: UIImage(systemName: "play.fill"), attributes: actionResumeEnabled ? [] : .disabled, state: .off, handler: { _ in
            self.resumeScan()
        }))
        
        // File menu
        let fileMenu = UIMenu(title: "File", options: .displayInline, children: fileMenuChildren)
        
        // Visibility menu
        let visibilityMenu = UIMenu(title: "Visibility...", children: [
            UIAction(title: "Status", image: statusShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState != .STATE_WELCOME) ? [] : .disabled, handler: { _ in
                self.statusShown = !self.statusShown
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Debug", image: debugShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState != .STATE_WELCOME) ? [] : .disabled, handler: { _ in
                self.debugShown = !self.debugShown
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Odom Visible", image: odomShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState == .STATE_MAPPING || self.mState == .STATE_CAMERA || self.mState == .STATE_VISUALIZING_CAMERA) ? [] : .disabled, handler: { _ in
                self.odomShown = !self.odomShown
                self.rtabmap!.setOdomCloudShown(shown: self.odomShown)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Graph Visibile", image: graphShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState == .STATE_MAPPING || self.mState == .STATE_CAMERA || self.mState == .STATE_IDLE) ? [] : .disabled, handler: { _ in
                self.graphShown = !self.graphShown
                self.rtabmap!.setGraphVisible(visible: self.graphShown)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Grid Visible", image: gridShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.gridShown = !self.gridShown
                self.rtabmap!.setGridVisible(visible: self.gridShown)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Optimized Graph", image: optimizedGraphShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState == .STATE_IDLE) ? [] : .disabled, handler: { _ in
                self.optimizedGraphShown = !self.optimizedGraphShown
                self.rtabmap!.setGraphOptimization(enabled: self.optimizedGraphShown)
                self.resetNoTouchTimer(true)
            })
        ])
        
        let settingsMenu = UIMenu(title: "Settings", options: .displayInline, children: [visibilityMenu,
            UIAction(title: "Settings", image: UIImage(systemName: "gearshape.2"), attributes: actionSettingsEnabled ? [] : .disabled, state: .off, handler: { _ in
                guard let settingsUrl = URL(string: UIApplication.openSettingsURLString) else {
                    return
                }

                if UIApplication.shared.canOpenURL(settingsUrl) {
                    UIApplication.shared.open(settingsUrl, completionHandler: { (success) in
                        print("Settings opened: \(success)") // Prints true
                    })
                }
            }),
            UIAction(title: "Restore All Default Settings", attributes: actionSettingsEnabled ? [] : .disabled, state: .off, handler: { _ in
                
                let ac = UIAlertController(title: "Reset All Default Settings", message: "Do you want to reset all settings to default?", preferredStyle: .alert)
                ac.addAction(UIAlertAction(title: "Yes", style: .default, handler: { _ in
                    let notificationCenter = NotificationCenter.default
                    notificationCenter.removeObserver(self)
                    UserDefaults.standard.reset()
                    self.registerSettingsBundle()
                    self.updateDisplayFromDefaults();
                    notificationCenter.addObserver(self, selector: #selector(self.defaultsChanged), name: UserDefaults.didChangeNotification, object: nil)
                }))
                ac.addAction(UIAlertAction(title: "No", style: .cancel, handler: nil))
                self.present(ac, animated: true)
             })
        ])

        menuButton.menu = UIMenu(title: "", children: [fileMenu, settingsMenu])
        menuButton.addTarget(self, action: #selector(ViewController.menuOpened(_:)), for: .menuActionTriggered)
        
        // Camera menu
        let renderingMenu = UIMenu(title: "Rendering", options: .displayInline, children: [
            UIAction(title: "Wireframe", image: self.wireframeShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.wireframeShown = !self.wireframeShown
                self.rtabmap!.setWireframe(enabled: self.wireframeShown)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Lighting", image: self.lightingShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: self.mState == .STATE_VISUALIZING || self.mState == .STATE_VISUALIZING_CAMERA || self.mState == .STATE_VISUALIZING_WHILE_LOADING ? [] : .disabled, handler: { _ in
                self.lightingShown = !self.lightingShown
                self.rtabmap!.setLighting(enabled: self.lightingShown)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Backface", image: self.backfaceShown ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.backfaceShown = !self.backfaceShown
                self.rtabmap!.setBackfaceCulling(enabled: !self.backfaceShown)
                self.resetNoTouchTimer(true)
            })
        ])
        
        let cameraMenu = UIMenu(title: "View", options: .displayInline, children: [
            UIAction(title: "First-P. View", image: cameraMode == 0 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: (self.mState == .STATE_CAMERA || self.mState == .STATE_VISUALIZING || self.mState == .STATE_MAPPING || self.mState == .STATE_VISUALIZING_CAMERA) ? [] : .disabled, handler: { _ in
                self.setGLCamera(type: 0)
                if(self.mState == .STATE_VISUALIZING)
                {
                    self.rtabmap?.setLocalizationMode(enabled: true)
                    self.rtabmap?.setPausedMapping(paused: false);
                    self.startCamera()
                    self.updateState(state: .STATE_VISUALIZING_CAMERA)
                }
                else
                {
                    self.resetNoTouchTimer(true)
                }
            }),
            UIAction(title: "Third-P. View", image: cameraMode == 1 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.setGLCamera(type: 1)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Top View", image: cameraMode == 2 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.setGLCamera(type: 2)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Ortho View", image: cameraMode == 3 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), handler: { _ in
                self.setGLCamera(type: 3)
                self.resetNoTouchTimer(true)
            })
        ])
        
        let showCloudMeshActions = mState != .STATE_VISUALIZING && mState != .STATE_VISUALIZING_CAMERA && mState != .STATE_PROCESSING && mState != .STATE_VISUALIZING_WHILE_LOADING
        let cloudMeshMenu = UIMenu(title: "CloudMesh", options: .displayInline, children: [
            UIAction(title: "Point Cloud", image: viewMode == 0 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: showCloudMeshActions ? [] : .disabled, handler: { _ in
                self.setMeshRendering(viewMode: 0)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Mesh", image: viewMode == 1 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: showCloudMeshActions ? [] : .disabled, handler: { _ in
                self.setMeshRendering(viewMode: 1)
                self.resetNoTouchTimer(true)
            }),
            UIAction(title: "Texture Mesh", image: viewMode == 2 ? UIImage(systemName: "checkmark.circle") : UIImage(systemName: "circle"), attributes: showCloudMeshActions ? [] : .disabled, handler: { _ in
                self.setMeshRendering(viewMode: 2)
                self.resetNoTouchTimer(true)
            })
        ])

        var viewMenuChildren: [UIMenuElement] = []
        viewMenuChildren.append(cameraMenu)
        viewMenuChildren.append(renderingMenu)
        viewMenuChildren.append(cloudMeshMenu)
        viewButton.menu = UIMenu(title: "", children: viewMenuChildren)
        viewButton.addTarget(self, action: #selector(ViewController.menuOpened(_:)), for: .menuActionTriggered)
    }
    
    @IBAction func menuOpened(_ sender:UIButton)
    {
        mMenuOpened = true;
    }
    
    func exportMesh(isOBJ: Bool)
    {
        let ac = UIAlertController(title: "Maximum Polygons", message: "\n\n\n\n\n\n\n\n\n\n", preferredStyle: .alert)
        ac.view.addSubview(maxPolygonsPickerView)
        maxPolygonsPickerView.selectRow(2, inComponent: 0, animated: false)
        ac.addAction(UIAlertAction(title: "OK", style: .default, handler: { _ in
            let pickerValue = self.maxPolygonsPickerData[self.maxPolygonsPickerView.selectedRow(inComponent: 0)]
            self.export(isOBJ: isOBJ, meshing: true, regenerateCloud: false, optimized: true, optimizedMaxPolygons: pickerValue*100000, previousState: self.mState);
        }))
        ac.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: nil))
        present(ac, animated: true)
    }
    
    func numberOfComponents(in pickerView: UIPickerView) -> Int {
        return 1
    }

    func pickerView(_ pickerView: UIPickerView, numberOfRowsInComponent component: Int) -> Int {
        return maxPolygonsPickerData.count
    }

    func pickerView(_ pickerView: UIPickerView, titleForRow row: Int, forComponent component: Int) -> String? {
        if(row == 0)
        {
            return "No Limit"
        }
        return "\(maxPolygonsPickerData[row])00 000"
    }
    
    // Auto-hide the home indicator to maximize immersion in AR experiences.
    override var prefersHomeIndicatorAutoHidden: Bool {
        return true
    }
    
    // Hide the status bar to maximize immersion in AR experiences.
    override var prefersStatusBarHidden: Bool {
        return !mHudVisible
    }

    //This is called when a new frame has been updated.
    func session(_ session: ARSession, didUpdate frame: ARFrame)
    {
        var status = ""
        var accept = false
        
        switch frame.camera.trackingState {
        case .normal:
            accept = true
        case .notAvailable:
            status = "Tracking not available"
        case .limited(.excessiveMotion):
            accept = true
            status = "Please Slow Your Movement"
        case .limited(.insufficientFeatures):
            accept = true
            status = "Avoid Featureless Surfaces"
        case .limited(.initializing):
            status = "Initializing"
        case .limited(.relocalizing):
            status = "Relocalizing"
        default:
            status = "Unknown tracking state"
        }
        
        mLastLightEstimate = frame.lightEstimate?.ambientIntensity
        
        if !status.isEmpty && mLastLightEstimate != nil && mLastLightEstimate! < 100 && accept {
            status = "Camera Is Occluded Or Lighting Is Too Dark"
        }

        if accept
        {
            if let rotation = UIApplication.shared.windows.first?.windowScene?.interfaceOrientation
            {
                rtabmap?.postOdometryEvent(frame: frame, orientation: rotation, viewport: self.view.frame.size)
            }
        }
        else
        {
            rtabmap?.notifyLost();
        }
        
        if !status.isEmpty {
            DispatchQueue.main.async {
                self.showToast(message: status, seconds: 2)
            }
        }
    }
    
    // This is called when a session fails.
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user.
        guard error is ARError else { return }
        let errorWithInfo = error as NSError
        let messages = [
            errorWithInfo.localizedDescription,
            errorWithInfo.localizedFailureReason,
            errorWithInfo.localizedRecoverySuggestion
        ]
        let errorMessage = messages.compactMap({ $0 }).joined(separator: "\n")
        DispatchQueue.main.async {
            // Present an alert informing about the error that has occurred.
            let alertController = UIAlertController(title: "The AR session failed.", message: errorMessage, preferredStyle: .alert)
            let restartAction = UIAlertAction(title: "Restart Session", style: .default) { _ in
                alertController.dismiss(animated: true, completion: nil)
                if let configuration = self.session.configuration {
                    self.session.run(configuration, options: [.resetSceneReconstruction, .resetTracking, .removeExistingAnchors])
                }
            }
            alertController.addAction(restartAction)
            self.present(alertController, animated: true, completion: nil)
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation])
    {
        mLastKnownLocation = locations.last!
        rtabmap?.setGPS(location: locations.last!);
    }
    
    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error)
    {
        print(error)
    }
    
    func locationManager(_ manager: CLLocationManager, didChangeAuthorization status: CLAuthorizationStatus)
    {
        print(status.rawValue)
        if(status == .notDetermined)
        {
            locationManager?.requestWhenInUseAuthorization()
        }
        if(status == .denied)
        {
            let alertController = UIAlertController(title: "GPS Disabled", message: "GPS option is enabled (Settings->Mapping...) but localization is denied for this App. To enable location for this App, go in Settings->Privacy->Location.", preferredStyle: .alert)

            let settingsAction = UIAlertAction(title: "Settings", style: .default) { (action) in
                self.locationManager = nil
                self.mLastKnownLocation = nil
                guard let settingsUrl = URL(string: UIApplication.openSettingsURLString) else {
                    return
                }

                if UIApplication.shared.canOpenURL(settingsUrl) {
                    UIApplication.shared.open(settingsUrl, completionHandler: { (success) in
                        print("Settings opened: \(success)") // Prints true
                    })
                }
            }
            alertController.addAction(settingsAction)
            
            let okAction = UIAlertAction(title: "Turn Off GPS", style: .default) { (action) in
                UserDefaults.standard.setValue(false, forKey: "SaveGPS")
                self.updateDisplayFromDefaults()
            }
            alertController.addAction(okAction)
            
            present(alertController, animated: true)
        }
        else if(status == .authorizedWhenInUse)
        {
            if locationManager != nil {
                if(locationManager!.accuracyAuthorization == .reducedAccuracy) {
                    let alertController = UIAlertController(title: "GPS Reduced Accuracy", message: "Your location settings for this App is set to reduced accuracy. We recommend to use high accuracy.", preferredStyle: .alert)

                    let settingsAction = UIAlertAction(title: "Settings", style: .default) { (action) in
                        guard let settingsUrl = URL(string: UIApplication.openSettingsURLString) else {
                            return
                        }

                        if UIApplication.shared.canOpenURL(settingsUrl) {
                            UIApplication.shared.open(settingsUrl, completionHandler: { (success) in
                                print("Settings opened: \(success)") // Prints true
                            })
                        }
                    }
                    alertController.addAction(settingsAction)
                    
                    let okAction = UIAlertAction(title: "Ignore", style: .default) { (action) in
                    }
                    alertController.addAction(okAction)
                    
                    present(alertController, animated: true)
                }
            }
        }
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
    
    var statusBarOrientation: UIInterfaceOrientation? {
        get {
            guard let orientation = UIApplication.shared.windows.first?.windowScene?.interfaceOrientation else {
                #if DEBUG
                fatalError("Could not obtain UIInterfaceOrientation from a valid windowScene")
                #else
                return nil
                #endif
            }
            return orientation
        }
    }
        
    deinit {
        EAGLContext.setCurrent(context)
        rtabmap = nil
        context = nil
        EAGLContext.setCurrent(nil)
    }
    
    var firstTouch: UITouch?
    var secondTouch: UITouch?
    
    override func touchesBegan(_ touches: Set<UITouch>,
                 with event: UIEvent?)
    {
        super.touchesBegan(touches, with: event)
        for touch in touches {
            if (firstTouch == nil) {
                firstTouch = touch
                let pose = touch.location(in: self.view)
                let normalizedX = pose.x / self.view.bounds.size.width;
                let normalizedY = pose.y / self.view.bounds.size.height;
                rtabmap?.onTouchEvent(touch_count: 1, event: 0, x0: Float(normalizedX), y0: Float(normalizedY), x1: 0.0, y1: 0.0);
            }
            else if (firstTouch != nil && secondTouch == nil)
            {
                secondTouch = touch
                if let pose0 = firstTouch?.location(in: self.view)
                {
                    if let pose1 = secondTouch?.location(in: self.view)
                    {
                        let normalizedX0 = pose0.x / self.view.bounds.size.width;
                        let normalizedY0 = pose0.y / self.view.bounds.size.height;
                        let normalizedX1 = pose1.x / self.view.bounds.size.width;
                        let normalizedY1 = pose1.y / self.view.bounds.size.height;
                        rtabmap?.onTouchEvent(touch_count: 2, event: 5, x0: Float(normalizedX0), y0: Float(normalizedY0), x1: Float(normalizedX1), y1: Float(normalizedY1));
                    }
                }
            }
        }
        if self.isPaused {
            self.view.setNeedsDisplay()
        }
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesMoved(touches, with: event)
        var firstTouchUsed = false
        var secondTouchUsed = false
        for touch in touches {
            if(touch == firstTouch)
            {
                firstTouchUsed = true
            }
            else if(touch == secondTouch)
            {
                secondTouchUsed = true
            }
        }
        if(secondTouch != nil)
        {
            if(firstTouchUsed || secondTouchUsed)
            {
                if let pose0 = firstTouch?.location(in: self.view)
                {
                    if let pose1 = secondTouch?.location(in: self.view)
                    {
                        let normalizedX0 = pose0.x / self.view.bounds.size.width;
                        let normalizedY0 = pose0.y / self.view.bounds.size.height;
                        let normalizedX1 = pose1.x / self.view.bounds.size.width;
                        let normalizedY1 = pose1.y / self.view.bounds.size.height;
                        rtabmap?.onTouchEvent(touch_count: 2, event: 2, x0: Float(normalizedX0), y0: Float(normalizedY0), x1: Float(normalizedX1), y1: Float(normalizedY1));
                    }
                }
            }
        }
        else if(firstTouchUsed)
        {
            if let pose = firstTouch?.location(in: self.view)
            {
                let normalizedX = pose.x / self.view.bounds.size.width;
                let normalizedY = pose.y / self.view.bounds.size.height;
                rtabmap?.onTouchEvent(touch_count: 1, event: 2, x0: Float(normalizedX), y0: Float(normalizedY), x1: 0.0, y1: 0.0);
            }
        }
        if self.isPaused {
            self.view.setNeedsDisplay()
        }
    }

    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesEnded(touches, with: event)
        for touch in touches {
            if(touch == firstTouch)
            {
                firstTouch = nil
            }
            else if(touch == secondTouch)
            {
                secondTouch = nil
            }
        }
        if (firstTouch == nil && secondTouch != nil)
        {
            firstTouch = secondTouch
            secondTouch = nil
        }
        if (firstTouch != nil && secondTouch == nil)
        {
            let pose = firstTouch!.location(in: self.view)
            let normalizedX = pose.x / self.view.bounds.size.width;
            let normalizedY = pose.y / self.view.bounds.size.height;
            rtabmap?.onTouchEvent(touch_count: 1, event: 0, x0: Float(normalizedX), y0: Float(normalizedY), x1: 0.0, y1: 0.0);
        }
        if self.isPaused {
            self.view.setNeedsDisplay()
        }
    }
    
    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        super.touchesCancelled(touches, with: event)
        for touch in touches {
            if(touch == firstTouch)
            {
                firstTouch = nil;
            }
            else if(touch == secondTouch)
            {
                secondTouch = nil;
            }
        }
        if self.isPaused {
            self.view.setNeedsDisplay()
        }
    }
    
    @IBAction func doubleTapped(_ gestureRecognizer: UITapGestureRecognizer) {
        if gestureRecognizer.state == UIGestureRecognizer.State.recognized
        {
            let pose = gestureRecognizer.location(in: gestureRecognizer.view)
            let normalizedX = pose.x / self.view.bounds.size.width;
            let normalizedY = pose.y / self.view.bounds.size.height;
            rtabmap?.onTouchEvent(touch_count: 3, event: 0, x0: Float(normalizedX), y0: Float(normalizedY), x1: 0.0, y1: 0.0);
        
            
            if self.isPaused {
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                    self.view.setNeedsDisplay()
                }
            }
        }
    }
    
    @IBAction func singleTapped(_ gestureRecognizer: UITapGestureRecognizer) {
        if gestureRecognizer.state == UIGestureRecognizer.State.recognized
        {
            resetNoTouchTimer(!mHudVisible)
            
            if self.isPaused {
                DispatchQueue.main.asyncAfter(deadline: .now() + 0.1) {
                    self.view.setNeedsDisplay()
                }
            }
        }
    }
    
    func registerSettingsBundle(){
        let appDefaults = [String:AnyObject]()
        UserDefaults.standard.register(defaults: appDefaults)
    }
    
    func updateDisplayFromDefaults()
    {
        //Get the defaults
        let defaults = UserDefaults.standard
 
        //let appendMode = defaults.bool(forKey: "AppendMode")
        
        // update preference
        rtabmap!.setOnlineBlending(enabled: defaults.bool(forKey: "Blending"));
        rtabmap!.setNodesFiltering(enabled: defaults.bool(forKey: "NodesFiltering"));
        rtabmap!.setRawScanSaved(enabled: defaults.bool(forKey: "SaveRawScan"));
        rtabmap!.setFullResolution(enabled: defaults.bool(forKey: "HDMode"));
        rtabmap!.setSmoothing(enabled: defaults.bool(forKey: "Smoothing"));
        rtabmap!.setAppendMode(enabled: defaults.bool(forKey: "AppendMode"));
        
        mTimeThr = (defaults.string(forKey: "TimeLimit")! as NSString).integerValue
        mMaxFeatures = (defaults.string(forKey: "MaxFeaturesExtractedLoopClosure")! as NSString).integerValue
        
        // Mapping parameters
        rtabmap!.setMappingParameter(key: "Rtabmap/DetectionRate", value: defaults.string(forKey: "UpdateRate")!);
        rtabmap!.setMappingParameter(key: "Rtabmap/TimeThr", value: defaults.string(forKey: "TimeLimit")!);
        rtabmap!.setMappingParameter(key: "Rtabmap/MemoryThr", value: defaults.string(forKey: "MemoryLimit")!);
        rtabmap!.setMappingParameter(key: "RGBD/LinearSpeedUpdate", value: defaults.string(forKey: "MaximumMotionSpeed")!);
        let motionSpeed = ((defaults.string(forKey: "MaximumMotionSpeed")!) as NSString).floatValue/2.0;
        rtabmap!.setMappingParameter(key: "RGBD/AngularSpeedUpdate", value: NSString(format: "%.2f", motionSpeed) as String);
        rtabmap!.setMappingParameter(key: "Rtabmap/LoopThr", value: defaults.string(forKey: "LoopClosureThreshold")!);
        rtabmap!.setMappingParameter(key: "Mem/RehearsalSimilarity", value: defaults.string(forKey: "SimilarityThreshold")!);
        rtabmap!.setMappingParameter(key: "Kp/MaxFeatures", value: defaults.string(forKey: "MaxFeaturesExtractedVocabulary")!);
        rtabmap!.setMappingParameter(key: "Vis/MaxFeatures", value: defaults.string(forKey: "MaxFeaturesExtractedLoopClosure")!);
        rtabmap!.setMappingParameter(key: "Vis/MinInliers", value: defaults.string(forKey: "MinInliers")!);
        rtabmap!.setMappingParameter(key: "RGBD/OptimizeMaxError", value: defaults.string(forKey: "MaxOptimizationError")!);
        rtabmap!.setMappingParameter(key: "Kp/DetectorStrategy", value: defaults.string(forKey: "FeatureType")!);
        rtabmap!.setMappingParameter(key: "Vis/FeatureType", value: defaults.string(forKey: "FeatureType")!);
        rtabmap!.setMappingParameter(key: "Mem/NotLinkedNodesKept", value: defaults.bool(forKey: "SaveAllFramesInDatabase") ? "true" : "false");
        rtabmap!.setMappingParameter(key: "RGBD/OptimizeFromGraphEnd", value: defaults.bool(forKey: "OptimizationfromGraphEnd") ? "true" : "false");
        rtabmap!.setMappingParameter(key: "RGBD/MaxOdomCacheSize", value: defaults.string(forKey: "MaximumOdometryCacheSize")!);
        rtabmap!.setMappingParameter(key: "Optimizer/Strategy", value: defaults.string(forKey: "GraphOptimizer")!);
        rtabmap!.setMappingParameter(key: "RGBD/ProximityBySpace", value: defaults.string(forKey: "ProximityDetection")!);

        let markerDetection = defaults.integer(forKey: "ArUcoMarkerDetection")
        if(markerDetection == -1)
        {
            rtabmap!.setMappingParameter(key: "RGBD/MarkerDetection", value: "false");
        }
        else
        {
            rtabmap!.setMappingParameter(key: "RGBD/MarkerDetection", value: "true");
            rtabmap!.setMappingParameter(key: "Marker/Dictionary", value: defaults.string(forKey: "ArUcoMarkerDetection")!);
            rtabmap!.setMappingParameter(key: "Marker/CornerRefinementMethod", value: (markerDetection > 16 ? "3":"0"));
            rtabmap!.setMappingParameter(key: "Marker/MaxDepthError", value: defaults.string(forKey: "MarkerDepthErrorEstimation")!);
            if let val = NumberFormatter().number(from: defaults.string(forKey: "MarkerSize")!)?.doubleValue
            {
                rtabmap!.setMappingParameter(key: "Marker/Length", value: String(format: "%f", val/100.0))
            }
            else{
                rtabmap!.setMappingParameter(key: "Marker/Length", value: "0")
            }
        }

        // Rendering
        rtabmap!.setCloudDensityLevel(value: defaults.integer(forKey: "PointCloudDensity"));
        rtabmap!.setMaxCloudDepth(value: defaults.float(forKey: "MaxDepth"));
        rtabmap!.setMinCloudDepth(value: defaults.float(forKey: "MinDepth"));
        rtabmap!.setDepthConfidence(value: defaults.integer(forKey: "DepthConfidence"));
        rtabmap!.setPointSize(value: defaults.float(forKey: "PointSize"));
        rtabmap!.setMeshAngleTolerance(value: defaults.float(forKey: "MeshAngleTolerance"));
        rtabmap!.setMeshTriangleSize(value: defaults.integer(forKey: "MeshTriangleSize"));
        rtabmap!.setMeshDecimationFactor(value: defaults.float(forKey: "MeshDecimationFactor"));
        let bgColor = defaults.float(forKey: "BackgroundColor");
        rtabmap!.setBackgroundColor(gray: bgColor);
        
        DispatchQueue.main.async {
            self.statusLabel.textColor = bgColor>=0.6 ? UIColor(white: 0.0, alpha: 1) : UIColor(white: 1.0, alpha: 1)
        }
    
        rtabmap!.setClusterRatio(value: defaults.float(forKey: "NoiseFilteringRatio"));
        rtabmap!.setMaxGainRadius(value: defaults.float(forKey: "ColorCorrectionRadius"));
        rtabmap!.setRenderingTextureDecimation(value: defaults.integer(forKey: "TextureResolution"));
        
        if(locationManager != nil && !defaults.bool(forKey: "SaveGPS"))
        {
            locationManager?.stopUpdatingLocation()
            locationManager = nil
            mLastKnownLocation = nil
        }
        else if(locationManager == nil && defaults.bool(forKey: "SaveGPS"))
        {
            locationManager = CLLocationManager()
            locationManager?.desiredAccuracy = kCLLocationAccuracyBestForNavigation
            locationManager?.delegate = self
        }
    }
    
    func resumeScan()
    {
        if(mState == State.STATE_VISUALIZING)
        {
            closeVisualization()
            rtabmap!.postExportation(visualize: false)
        }
        
        let alertController = UIAlertController(title: "Append Mode", message: "The camera preview will not be aligned to map on start, move to a previously scanned area, then push Record. When a loop closure is detected, new scans will be appended to map.", preferredStyle: .alert)

        let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
        }
        alertController.addAction(okAction)
        
        present(alertController, animated: true)
        
        setGLCamera(type: 0);
        startCamera();
    }
    
    func newScan()
    {
        print("databases.size() = \(databases.size())")
        if(databases.count >= 5 && !mReviewRequested && self.depthSupported)
        {
            SKStoreReviewController.requestReviewInCurrentScene()
            mReviewRequested = true
        }
        
        if(mState == State.STATE_VISUALIZING)
        {
            closeVisualization()
        }
        
        mMapNodes = 0;
        self.openedDatabasePath = nil
        let tmpDatabase = self.getDocumentDirectory().appendingPathComponent(self.RTABMAP_TMP_DB)
        let inMemory = UserDefaults.standard.bool(forKey: "DatabaseInMemory")
        if(!(self.mState == State.STATE_CAMERA || self.mState == State.STATE_MAPPING) &&
           FileManager.default.fileExists(atPath: tmpDatabase.path) &&
           tmpDatabase.fileSize > 1024*1024) // > 1MB
        {
            dismiss(animated: true, completion: {
                let msg = "The previous session (\(tmpDatabase.fileSizeString)) was not correctly saved, do you want to recover it?"
                let alert = UIAlertController(title: "Recovery", message: msg, preferredStyle: .alert)
                let alertActionNo = UIAlertAction(title: "Ignore", style: .destructive) {
                    (UIAlertAction) -> Void in
                    do {
                        try FileManager.default.removeItem(at: tmpDatabase)
                    }
                    catch {
                        print("Could not clear tmp database: \(error)")
                    }
                    self.newScan()
                }
                alert.addAction(alertActionNo)
                let alertActionCancel = UIAlertAction(title: "Cancel", style: .cancel) {
                    (UIAlertAction) -> Void in
                    // do nothing
                }
                alert.addAction(alertActionCancel)
                let alertActionYes = UIAlertAction(title: "Yes", style: .default) {
                    (UIAlertAction2) -> Void in

                    let fileName = Date().getFormattedDate(format: "yyMMdd-HHmmss") + ".db"
                    let outputDbPath = self.getDocumentDirectory().appendingPathComponent(fileName).path
                    
                    var indicator: UIActivityIndicatorView?
                    
                    let alertView = UIAlertController(title: "Recovering", message: "Please wait while recovering data...", preferredStyle: .alert)
                    let alertViewActionCancel = UIAlertAction(title: "Cancel", style: .cancel) {
                        (UIAlertAction) -> Void in
                        self.dismiss(animated: true, completion: {
                            self.progressView = nil
                            
                            indicator = UIActivityIndicatorView(style: .large)
                            indicator?.frame = CGRect(x: 0.0, y: 0.0, width: 60.0, height: 60.0)
                            indicator?.center = self.view.center
                            self.view.addSubview(indicator!)
                            indicator?.bringSubviewToFront(self.view)
                            
                            indicator?.startAnimating()
                            self.rtabmap!.cancelProcessing();
                        })
                    }
                    alertView.addAction(alertViewActionCancel)
                    
                    let previousState = self.mState
                    self.updateState(state: .STATE_PROCESSING);
                    
                    self.present(alertView, animated: true, completion: {
                        //  Add your progressbar after alert is shown (and measured)
                        let margin:CGFloat = 8.0
                        let rect = CGRect(x: margin, y: 84.0, width: alertView.view.frame.width - margin * 2.0 , height: 2.0)
                        self.progressView = UIProgressView(frame: rect)
                        self.progressView!.progress = 0
                        self.progressView!.tintColor = self.view.tintColor
                        alertView.view.addSubview(self.progressView!)
                        
                        var success : Bool = false
                        DispatchQueue.background(background: {
                            
                            success = self.rtabmap!.recover(from: tmpDatabase.path, to: outputDbPath)
                            
                        }, completion:{
                            if(indicator != nil)
                            {
                                indicator!.stopAnimating()
                                indicator!.removeFromSuperview()
                            }
                            if self.progressView != nil
                            {
                                self.dismiss(animated: self.openedDatabasePath == nil, completion: {
                                    if(success)
                                    {
                                        let alertSaved = UIAlertController(title: "Database saved!", message: String(format: "Database \"%@\" successfully recovered!", fileName), preferredStyle: .alert)
                                        let yes = UIAlertAction(title: "OK", style: .default) {
                                            (UIAlertAction) -> Void in
                                            self.openDatabase(fileUrl: URL(fileURLWithPath: outputDbPath))
                                        }
                                        alertSaved.addAction(yes)
                                        self.present(alertSaved, animated: true, completion: nil)
                                    }
                                    else
                                    {
                                        self.updateState(state: previousState);
                                        self.showToast(message: "Recovery failed!", seconds: 4)
                                    }
                                })
                            }
                            else
                            {
                                self.showToast(message: "Recovery canceled", seconds: 2)
                                self.updateState(state: previousState);
                            }
                        })
                    })
                }
                alert.addAction(alertActionYes)
                self.present(alert, animated: true, completion: nil)
            })
        }
        else
        {
            self.rtabmap!.openDatabase(databasePath: tmpDatabase.path, databaseInMemory: inMemory, optimize: false, clearDatabase: true)

            if(!(self.mState == State.STATE_CAMERA || self.mState == State.STATE_MAPPING))
            {
                self.setGLCamera(type: 0);
                self.startCamera();
            }
        }
    }
    
    func save()
    {
        //Step : 1
        let alert = UIAlertController(title: "Save Scan", message: "RTAB-Map Database Name (*.db):", preferredStyle: .alert )
        //Step : 2
        let save = UIAlertAction(title: "Save", style: .default) { (alertAction) in
            let textField = alert.textFields![0] as UITextField
            if textField.text != "" {
                //Read TextFields text data
                let fileName = textField.text!+".db"
                let filePath = self.getDocumentDirectory().appendingPathComponent(fileName).path
                if FileManager.default.fileExists(atPath: filePath) {
                    let alert = UIAlertController(title: "File Already Exists", message: "Do you want to overwrite the existing file?", preferredStyle: .alert)
                    let yes = UIAlertAction(title: "Yes", style: .default) {
                        (UIAlertAction) -> Void in
                        self.saveDatabase(fileName: fileName);
                    }
                    alert.addAction(yes)
                    let no = UIAlertAction(title: "No", style: .cancel) {
                        (UIAlertAction) -> Void in
                    }
                    alert.addAction(no)
                    
                    self.present(alert, animated: true, completion: nil)
                } else {
                    self.saveDatabase(fileName: fileName);
                }
            }
        }

        //Step : 3
        var placeholder = Date().getFormattedDate(format: "yyMMdd-HHmmss")
        if self.openedDatabasePath != nil && !self.openedDatabasePath!.path.isEmpty
        {
            var components = self.openedDatabasePath!.lastPathComponent.components(separatedBy: ".")
            if components.count > 1 { // If there is a file extension
                components.removeLast()
                placeholder = components.joined(separator: ".")
            } else {
                placeholder = self.openedDatabasePath!.lastPathComponent
            }
        }
        alert.addTextField { (textField) in
                textField.text = placeholder
        }

        //Step : 4
        alert.addAction(save)
        //Cancel action
        alert.addAction(UIAlertAction(title: "Cancel", style: .default) { (alertAction) in })

        self.present(alert, animated: true) {
            alert.textFields?.first?.selectAll(nil)
        }
    }
    
    func saveDatabase(fileName: String)
    {
        let filePath = self.getDocumentDirectory().appendingPathComponent(fileName).path
        
        let indicator: UIActivityIndicatorView = UIActivityIndicatorView(style: .large)
        indicator.frame = CGRect(x: 0.0, y: 0.0, width: 60.0, height: 60.0)
        indicator.center = view.center
        view.addSubview(indicator)
        indicator.bringSubviewToFront(view)
        
        indicator.startAnimating()
        
        let previousState = mState;
        updateState(state: .STATE_PROCESSING);
        
        DispatchQueue.background(background: {
            self.rtabmap?.save(databasePath: filePath); // save
        }, completion:{
            // main thread
            indicator.stopAnimating()
            indicator.removeFromSuperview()
            
            self.openedDatabasePath = URL(fileURLWithPath: filePath)
            
            let alert = UIAlertController(title: "Database saved!", message: String(format: "Database \"%@\" successfully saved!", fileName), preferredStyle: .alert)
            let yes = UIAlertAction(title: "OK", style: .default) {
                (UIAlertAction) -> Void in
            }
            alert.addAction(yes)
            self.present(alert, animated: true, completion: nil)
            do {
                let tmpDatabase = self.getDocumentDirectory().appendingPathComponent(self.RTABMAP_TMP_DB)
                try FileManager.default.removeItem(at: tmpDatabase)
            }
            catch {
                print("Could not clear tmp database: \(error)")
            }
            self.updateDatabases()
            self.updateState(state: previousState)
        })
    }
    
    private func export(isOBJ: Bool, meshing: Bool, regenerateCloud: Bool, optimized: Bool, optimizedMaxPolygons: Int, previousState: State)
    {
        let defaults = UserDefaults.standard
        let cloudVoxelSize = defaults.float(forKey: "VoxelSize")
        let textureSize = isOBJ ? defaults.integer(forKey: "TextureSize") : 0
        let textureCount = defaults.integer(forKey: "MaximumOutputTextures")
        let normalK = defaults.integer(forKey: "NormalK")
        let maxTextureDistance = defaults.float(forKey: "MaxTextureDistance")
        let minTextureClusterSize = defaults.integer(forKey: "MinTextureClusterSize")
        let optimizedVoxelSize = cloudVoxelSize
        let optimizedDepth = defaults.integer(forKey: "ReconstructionDepth")
        let optimizedColorRadius = defaults.float(forKey: "ColorRadius")
        let optimizedCleanWhitePolygons = defaults.bool(forKey: "CleanMesh")
        let optimizedMinClusterSize = defaults.integer(forKey: "PolygonFiltering")
        let blockRendering = false
        
        var indicator: UIActivityIndicatorView?
        
        let alertView = UIAlertController(title: "Assembling", message: "Please wait while assembling data...", preferredStyle: .alert)
        alertView.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: { _ in
            self.dismiss(animated: true, completion: {
                self.progressView = nil
                
                indicator = UIActivityIndicatorView(style: .large)
                indicator?.frame = CGRect(x: 0.0, y: 0.0, width: 60.0, height: 60.0)
                indicator?.center = self.view.center
                self.view.addSubview(indicator!)
                indicator?.bringSubviewToFront(self.view)
                
                indicator?.startAnimating()
                
                self.rtabmap!.cancelProcessing()
            })
            
        }))

        updateState(state: .STATE_PROCESSING);
        
        present(alertView, animated: true, completion: {
            //  Add your progressbar after alert is shown (and measured)
            let margin:CGFloat = 8.0
            let rect = CGRect(x: margin, y: 84.0, width: alertView.view.frame.width - margin * 2.0 , height: 2.0)
            self.progressView = UIProgressView(frame: rect)
            self.progressView!.progress = 0
            self.progressView!.tintColor = self.view.tintColor
            alertView.view.addSubview(self.progressView!)
            
            var success : Bool = false
            DispatchQueue.background(background: {
                
                success = self.rtabmap!.exportMesh(
                    cloudVoxelSize: cloudVoxelSize,
                    regenerateCloud: regenerateCloud,
                    meshing: meshing,
                    textureSize: textureSize,
                    textureCount: textureCount,
                    normalK: normalK,
                    optimized: optimized,
                    optimizedVoxelSize: optimizedVoxelSize,
                    optimizedDepth: optimizedDepth,
                    optimizedMaxPolygons: optimizedMaxPolygons,
                    optimizedColorRadius: optimizedColorRadius,
                    optimizedCleanWhitePolygons: optimizedCleanWhitePolygons,
                    optimizedMinClusterSize: optimizedMinClusterSize,
                    optimizedMaxTextureDistance: maxTextureDistance,
                    optimizedMinTextureClusterSize: minTextureClusterSize,
                    blockRendering: blockRendering)
                
            }, completion:{
                if(indicator != nil)
                {
                    indicator!.stopAnimating()
                    indicator!.removeFromSuperview()
                }
                if self.progressView != nil
                {
                    self.dismiss(animated: self.openedDatabasePath == nil, completion: {
                        if(success)
                        {
                            if(!meshing && cloudVoxelSize>0.0)
                            {
                                self.showToast(message: "Cloud assembled and voxelized at \(cloudVoxelSize) m.", seconds: 2)
                            }
                            
                            if(!meshing)
                            {
                                self.setMeshRendering(viewMode: 0)
                            }
                            else if(!isOBJ)
                            {
                                self.setMeshRendering(viewMode: 1)
                            }
                            else // isOBJ
                            {
                                self.setMeshRendering(viewMode: 2)
                            }

                            self.updateState(state: .STATE_VISUALIZING)
                            
                            self.rtabmap!.postExportation(visualize: true)

                            self.setGLCamera(type: 2)

                            if self.openedDatabasePath == nil
                            {
                                self.save();
                            }
                        }
                        else
                        {
                            self.updateState(state: previousState);
                            self.showToast(message: "Exporting map failed!", seconds: 4)
                        }
                    })
                }
                else
                {
                    self.showToast(message: "Export canceled", seconds: 2)
                    self.updateState(state: previousState);
                }
            })
        })
    }
    
    private func optimization(withStandardMeshExport: Bool = false, approach: Int)
    {
        if(mState == State.STATE_VISUALIZING)
        {
            closeVisualization()
            rtabmap!.postExportation(visualize: false)
        }
        
        let alertView = UIAlertController(title: "Post-Processing", message: "Please wait while optimizing...", preferredStyle: .alert)
        alertView.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: { _ in
            self.dismiss(animated: true)
            self.progressView = nil
            self.rtabmap!.cancelProcessing()
        }))

        let previousState = mState
        
        updateState(state: .STATE_PROCESSING)
        
        //  Show it to your users
        present(alertView, animated: true, completion: {
            //  Add your progressbar after alert is shown (and measured)
            let margin:CGFloat = 8.0
            let rect = CGRect(x: margin, y: 72.0, width: alertView.view.frame.width - margin * 2.0 , height: 2.0)
            self.progressView = UIProgressView(frame: rect)
            self.progressView!.progress = 0
            self.progressView!.tintColor = self.view.tintColor
            alertView.view.addSubview(self.progressView!)
            
            var loopDetected : Int = -1
            DispatchQueue.background(background: {
                loopDetected = self.rtabmap!.postProcessing(approach: approach);
            }, completion:{
                // main thread
                if self.progressView != nil
                {
                    self.dismiss(animated: self.openedDatabasePath == nil, completion: {
                        self.progressView = nil
                        
                        if(loopDetected >= 0)
                        {
                            if(approach  == -1)
                            {
                                if(withStandardMeshExport)
                                {
                                    self.export(isOBJ: true, meshing: true, regenerateCloud: false, optimized: true, optimizedMaxPolygons: 200000, previousState: previousState);
                                }
                                else
                                {
                                    if self.openedDatabasePath == nil
                                    {
                                        self.save();
                                    }
                                }
                            }
                        }
                        else if(loopDetected < 0)
                        {
                            self.showToast(message: "Optimization failed!", seconds: 4.0)
                        }
                    })
                }
                else
                {
                    self.showToast(message: "Optimization canceled", seconds: 4.0)
                }
                self.updateState(state: .STATE_IDLE);
            })
        })
    }
    
    func stopMapping(ignoreSaving: Bool)
    {
        session.pause()
        locationManager?.stopUpdatingLocation()
        rtabmap?.setPausedMapping(paused: true)
        rtabmap?.stopCamera()
        setGLCamera(type: 2)
        if(mState == .STATE_VISUALIZING_CAMERA)
        {
            self.rtabmap?.setLocalizationMode(enabled: false)
        }
        updateState(state: mState == .STATE_VISUALIZING_CAMERA ? .STATE_VISUALIZING : .STATE_IDLE);
        
        if !ignoreSaving
        {
            dismiss(animated: true, completion: {
                var msg = "Do you want to do standard graph optimization and make a nice assembled mesh now? This can be also done later using \"Optimize\" and \"Assemble\" menus."
                let depthUsed = self.depthSupported && UserDefaults.standard.bool(forKey: "LidarMode")
                if !depthUsed
                {
                    msg = "Do you want to do standard graph optimization now? This can be also done later using \"Optimize\" menu."
                }
                let alert = UIAlertController(title: "Mapping Stopped! Optimize Now?", message: msg, preferredStyle: .alert)
                if depthUsed {
                    let alertActionOnlyGraph = UIAlertAction(title: "Only Optimize", style: .default)
                    {
                        (UIAlertAction) -> Void in
                        self.optimization(withStandardMeshExport: false, approach: -1)
                    }
                    alert.addAction(alertActionOnlyGraph)
                }
                let alertActionNo = UIAlertAction(title: "Save First", style: .cancel) {
                    (UIAlertAction) -> Void in
                    self.save()
                }
                alert.addAction(alertActionNo)
                let alertActionYes = UIAlertAction(title: "Yes", style: .default) {
                    (UIAlertAction) -> Void in
                    self.optimization(withStandardMeshExport: depthUsed, approach: -1)
                }
                alert.addAction(alertActionYes)
                self.present(alert, animated: true, completion: nil)
            })
        }
        else if(mMapNodes == 0)
        {
            updateState(state: State.STATE_WELCOME);
            statusLabel.text = ""
        }
    }
    func shareFile(_ fileUrl: URL) {
        let fileURL = NSURL(fileURLWithPath: fileUrl.path)

        // Create the Array which includes the files you want to share
        var filesToShare = [Any]()

        // Add the path of the file to the Array
        filesToShare.append(fileURL)

        // Make the activityViewContoller which shows the share-view
        let activityViewController = UIActivityViewController(activityItems: filesToShare, applicationActivities: nil)
        
        if let popoverController = activityViewController.popoverPresentationController {
            popoverController.sourceRect = CGRect(x: UIScreen.main.bounds.width / 2, y: UIScreen.main.bounds.height / 2, width: 0, height: 0)
            popoverController.sourceView = self.view
            popoverController.permittedArrowDirections = UIPopoverArrowDirection(rawValue: 0)
        }

        // Show the share-view
        self.present(activityViewController, animated: true, completion: nil)
    }
    
    func openDatabase(fileUrl: URL) {
        
        if(mState == .STATE_CAMERA) {
            stopMapping(ignoreSaving: true)
        }
        
        openedDatabasePath = fileUrl;
        let fileName: String = self.openedDatabasePath!.lastPathComponent
        
        var progressDialog = UIAlertController(title: "Loading", message: String(format: "Loading \"%@\". Please wait while point clouds and/or meshes are created...", fileName), preferredStyle: .alert)
        
        //  Show it to your users
        self.present(progressDialog, animated: true)

        updateState(state: .STATE_PROCESSING);
        var status = 0
        DispatchQueue.background(background: {
            status = self.rtabmap!.openDatabase(databasePath: self.openedDatabasePath!.path, databaseInMemory: true, optimize: false, clearDatabase: false)
        }, completion:{
            // main thread
            if(status == -1) {
                self.dismiss(animated: true)
                self.showToast(message: "The map is loaded but optimization of the map's graph has failed, so the map cannot be shown. Change the Graph Optimizer approach used or enable/disable if the graph is optimized from graph end in \"Settings -> Mapping...\" and try opening again.", seconds: 4)
            }
            else if(status == -2) {
                self.dismiss(animated: true)
                self.showToast(message: "Failed to open database: Out of memory! Try again after lowering Point Cloud Density in Settings.", seconds: 4)
            }
            else {
                if(status >= 1 && status<=3) {
                    self.updateState(state: .STATE_VISUALIZING);
                    self.resetNoTouchTimer(true);
                }
                else {
                    self.setGLCamera(type: 2);
                    self.updateState(state: .STATE_IDLE);
                    self.dismiss(animated: true)
                    self.showToast(message: "Database loaded!", seconds: 2)
                }
            }
            
        })
    }
    
    func closeVisualization()
    {
        updateState(state: .STATE_IDLE);
    }
    
    func rename(fileURL: URL)
    {
        //Step : 1
        let alert = UIAlertController(title: "Rename Scan", message: "RTAB-Map Database Name (*.db):", preferredStyle: .alert )
        //Step : 2
        let rename = UIAlertAction(title: "Rename", style: .default) { (alertAction) in
            let textField = alert.textFields![0] as UITextField
            if textField.text != "" {
                //Read TextFields text data
                let fileName = textField.text!+".db"
                let filePath = self.getDocumentDirectory().appendingPathComponent(fileName).path
                if FileManager.default.fileExists(atPath: filePath) {
                    let alert = UIAlertController(title: "File Already Exists", message: "Do you want to overwrite the existing file?", preferredStyle: .alert)
                    let yes = UIAlertAction(title: "Yes", style: .default) {
                        (UIAlertAction) -> Void in
                        
                        do {
                            try FileManager.default.moveItem(at: fileURL, to: URL(fileURLWithPath: filePath))
                            print("File \(fileURL) renamed to \(filePath)")
                        }
                        catch {
                            print("Error renaming file \(fileURL) to \(filePath)")
                        }
                        self.openLibrary()
                    }
                    alert.addAction(yes)
                    let no = UIAlertAction(title: "No", style: .cancel) {
                        (UIAlertAction) -> Void in
                    }
                    alert.addAction(no)
                    
                    self.present(alert, animated: true, completion: nil)
                } else {
                    do {
                        try FileManager.default.moveItem(at: fileURL, to: URL(fileURLWithPath: filePath))
                        print("File \(fileURL) renamed to \(filePath)")
                    }
                    catch {
                        print("Error renaming file \(fileURL) to \(filePath)")
                    }
                    self.openLibrary()
                }
            }
        }

        //Step : 3
        alert.addTextField { (textField) in
            var components = fileURL.lastPathComponent.components(separatedBy: ".")
            if components.count > 1 { // If there is a file extension
              components.removeLast()
                textField.text = components.joined(separator: ".")
            } else {
                textField.text = fileURL.lastPathComponent
            }
        }

        //Step : 4
        alert.addAction(rename)
        //Cancel action
        alert.addAction(UIAlertAction(title: "Cancel", style: .default) { (alertAction) in })

        self.present(alert, animated: true) {
            alert.textFields?.first?.selectAll(nil)
        }
    }
    
    func exportOBJPLY()
    {
        //Step : 1
        let alert = UIAlertController(title: "Export Scan", message: "Model Name:", preferredStyle: .alert )
        //Step : 2
        let save = UIAlertAction(title: "Ok", style: .default) { (alertAction) in
            let textField = alert.textFields![0] as UITextField
            if textField.text != "" {
                self.dismiss(animated: true)
                //Read TextFields text data
                let fileName = textField.text!+".zip"
                let filePath = self.getDocumentDirectory().appendingPathComponent(fileName).path
                if FileManager.default.fileExists(atPath: filePath) {
                    let alert = UIAlertController(title: "File Already Exists", message: "Do you want to overwrite the existing file?", preferredStyle: .alert)
                    let yes = UIAlertAction(title: "Yes", style: .default) {
                        (UIAlertAction) -> Void in
                        self.writeExportedFiles(fileName: textField.text!);
                    }
                    alert.addAction(yes)
                    let no = UIAlertAction(title: "No", style: .cancel) {
                        (UIAlertAction) -> Void in
                    }
                    alert.addAction(no)
                    
                    self.present(alert, animated: true, completion: nil)
                } else {
                    self.writeExportedFiles(fileName: textField.text!);
                }
            }
        }

        //Step : 3
        alert.addTextField { (textField) in
            if self.openedDatabasePath != nil && !self.openedDatabasePath!.path.isEmpty
            {
                var components = self.openedDatabasePath!.lastPathComponent.components(separatedBy: ".")
                if components.count > 1 { // If there is a file extension
                    components.removeLast()
                    textField.text = components.joined(separator: ".")
                } else {
                    textField.text = self.openedDatabasePath!.lastPathComponent
                }
            }
            else {
                textField.text = Date().getFormattedDate(format: "yyMMdd-HHmmss")
            }
        }

        //Step : 4
        alert.addAction(save)
        //Cancel action
        alert.addAction(UIAlertAction(title: "Cancel", style: .cancel) { (alertAction) in })

        self.present(alert, animated: true) {
            alert.textFields?.first?.selectAll(nil)
        }
    }

    func writeExportedFiles(fileName: String)
    {
        let alertView = UIAlertController(title: "Exporting", message: "Please wait while zipping data to \(fileName+".zip")...", preferredStyle: .alert)
        alertView.addAction(UIAlertAction(title: "Cancel", style: .cancel, handler: { _ in
            self.dismiss(animated: true)
            self.progressView = nil
            self.rtabmap!.cancelProcessing()
        }))
        
        let previousState = mState;

        updateState(state: .STATE_PROCESSING);
        
        present(alertView, animated: true, completion: {
            //  Add your progressbar after alert is shown (and measured)
            let margin:CGFloat = 8.0
            let rect = CGRect(x: margin, y: 84.0, width: alertView.view.frame.width - margin * 2.0 , height: 2.0)
            self.progressView = UIProgressView(frame: rect)
            self.progressView!.progress = 0
            self.progressView!.tintColor = self.view.tintColor
            alertView.view.addSubview(self.progressView!)
            
            let exportDir = self.getTmpDirectory().appendingPathComponent(self.RTABMAP_EXPORT_DIR)
           
            do {
                try FileManager.default.removeItem(at: exportDir)
            }
            catch
            {}
            
            do {
                try FileManager.default.createDirectory(at: exportDir, withIntermediateDirectories: true)
            }
            catch
            {
                print("Failed adding export directory \(exportDir)")
                return
            }
            
            var success : Bool = false
            var zipFileUrl : URL!
            DispatchQueue.background(background: {
                print("Exporting to directory \(exportDir.path) with name \(fileName)")
                if(self.rtabmap!.writeExportedMesh(directory: exportDir.path, name: fileName))
                {
                    do {
                        let fileURLs = try FileManager.default.contentsOfDirectory(at: exportDir, includingPropertiesForKeys: nil)
                        if(!fileURLs.isEmpty)
                        {
                            do {
                                zipFileUrl = try Zip.quickZipFiles(fileURLs, fileName: fileName) // Zip
                                print("Zip file \(zipFileUrl.path) created (size=\(zipFileUrl.fileSizeString)")
                                success = true
                            }
                            catch {
                              print("Something went wrong while zipping")
                            }
                        }
                    } catch {
                        print("No files exported to \(exportDir)")
                        return
                    }
                }
                
            }, completion:{
                if self.progressView != nil
                {
                    self.dismiss(animated: true)
                }
                if(success)
                {
                    let alertShare = UIAlertController(title: "Mesh/Cloud Saved!", message: "\(fileName+".zip") (\(zipFileUrl.fileSizeString) successfully exported in Documents of RTAB-Map! Share it?", preferredStyle: .alert)
                    let alertActionYes = UIAlertAction(title: "Yes", style: .default) {
                        (UIAlertAction) -> Void in
                        self.shareFile(zipFileUrl)
                    }
                    alertShare.addAction(alertActionYes)
                    let alertActionNo = UIAlertAction(title: "No", style: .cancel) {
                        (UIAlertAction) -> Void in
                       
                    }
                    alertShare.addAction(alertActionNo)
                    
                    self.present(alertShare, animated: true, completion: nil)
                }
                else
                {
                    self.showToast(message: "Exporting mesh/cloud canceled!", seconds: 2)
                }
                self.updateState(state: previousState);
            })
        })
    }
    
    func updateDatabases()
    {
        databases.removeAll()
        do {
            let fileURLs = try FileManager.default.contentsOfDirectory(at: getDocumentDirectory(), includingPropertiesForKeys: nil)
            // if you want to filter the directory contents you can do like this:
            
            let data = fileURLs.map { url in
                        (url, (try? url.resourceValues(forKeys: [.contentModificationDateKey]))?.contentModificationDate ?? Date.distantPast)
                    }
                    .sorted(by: { $0.1 > $1.1 }) // sort descending modification dates
                    .map { $0.0 } // extract file names
            databases = data.filter{ $0.pathExtension == "db" && $0.lastPathComponent != RTABMAP_TMP_DB && $0.lastPathComponent != RTABMAP_RECOVERY_DB }
            
        } catch {
            print("Error while enumerating files : \(error.localizedDescription)")
            return
        }
    }
    
    func openLibrary()
    {
        updateDatabases();
        
        if databases.isEmpty {
            return
        }
        
        let alertController = UIAlertController(title: "Library", message: nil, preferredStyle: .alert)
        let customView = VerticalScrollerView()
        customView.dataSource = self
        customView.delegate = self
        customView.reload()
        alertController.view.addSubview(customView)
        customView.translatesAutoresizingMaskIntoConstraints = false
        customView.topAnchor.constraint(equalTo: alertController.view.topAnchor, constant: 60).isActive = true
        customView.rightAnchor.constraint(equalTo: alertController.view.rightAnchor, constant: -10).isActive = true
        customView.leftAnchor.constraint(equalTo: alertController.view.leftAnchor, constant: 10).isActive = true
        customView.bottomAnchor.constraint(equalTo: alertController.view.bottomAnchor, constant: -45).isActive = true
        
        alertController.view.translatesAutoresizingMaskIntoConstraints = false
        alertController.view.heightAnchor.constraint(equalToConstant: 600).isActive = true
        alertController.view.widthAnchor.constraint(equalToConstant: 400).isActive = true

        customView.backgroundColor = .darkGray

        let selectAction = UIAlertAction(title: "Select", style: .default) { (action) in
            self.openDatabase(fileUrl: self.databases[self.currentDatabaseIndex])
        }
        
        let cancelAction = UIAlertAction(title: "Cancel", style: .cancel, handler: nil)
        alertController.addAction(selectAction)
        alertController.addAction(cancelAction)
        self.present(alertController, animated: true, completion: nil)
    }

    //MARK: Actions   
    @IBAction func stopAction(_ sender: UIButton) {
        stopMapping(ignoreSaving: false)
    }

    @IBAction func recordAction(_ sender: UIButton) {
        rtabmap?.setPausedMapping(paused: false);
        updateState(state: .STATE_MAPPING)
    }
    
    @IBAction func newScanAction(_ sender: UIButton) {
        newScan()
    }
    
    @IBAction func closeVisualizationAction(_ sender: UIButton) {
        closeVisualization()
        rtabmap!.postExportation(visualize: false)
    }
    
    @IBAction func stopCameraAction(_ sender: UIButton) {
        appMovedToBackground();
    }
    
    @IBAction func exportOBJPLYAction(_ sender: UIButton) {
        exportOBJPLY()
    }
    
    @IBAction func libraryAction(_ sender: UIButton) {
        openLibrary();
    }
    @IBAction func rotateGridAction(_ sender: UISlider) {
        rtabmap!.setGridRotation((Float(sender.value)-90.0)/2.0)
        self.view.setNeedsDisplay()
    }
    @IBAction func clipDistanceAction(_ sender: UISlider) {
        rtabmap!.setOrthoCropFactor(Float(120-sender.value)/20.0 - 3.0)
        self.view.setNeedsDisplay()
    }
}

func clearBackgroundColor(of view: UIView) {
    if let effectsView = view as? UIVisualEffectView {
        effectsView.removeFromSuperview()
        return
    }

    view.backgroundColor = .clear
    view.subviews.forEach { (subview) in
        clearBackgroundColor(of: subview)
    }
}

extension ViewController: GLKViewControllerDelegate {
    
    // OPENGL UPDATE
    func glkViewControllerUpdate(_ controller: GLKViewController) {
        
    }
    
    // OPENGL DRAW
    override func glkView(_ view: GLKView, drawIn rect: CGRect) {
        if let rotation = UIApplication.shared.windows.first?.windowScene?.interfaceOrientation
        {
            let viewportSize = CGSize(width: rect.size.width * view.contentScaleFactor, height: rect.size.height * view.contentScaleFactor)
            rtabmap?.setupGraphic(size: viewportSize, orientation: rotation)
        }
        
        let value = rtabmap?.render()
        
        DispatchQueue.main.async {
            if(value != 0 && self.progressView != nil)
            {
                print("Render dismissing")
                self.dismiss(animated: true)
                self.progressView = nil
            }
            if(value == -1)
            {
                self.showToast(message: "Out of Memory!", seconds: 2)
            }
            else if(value == -2)
            {
                self.showToast(message: "Rendering Error!", seconds: 2)
            }
        }
    }
}

extension Date {
   func getFormattedDate(format: String) -> String {
        let dateformat = DateFormatter()
        dateformat.dateFormat = format
        return dateformat.string(from: self)
    }
    
    var millisecondsSince1970:Int64 {
        Int64((self.timeIntervalSince1970 * 1000.0).rounded())
    }
    
    init(milliseconds:Int64) {
        self = Date(timeIntervalSince1970: TimeInterval(milliseconds) / 1000)
    }
}

extension DispatchQueue {

    static func background(delay: Double = 0.0, background: (()->Void)? = nil, completion: (() -> Void)? = nil) {
        DispatchQueue.global(qos: .userInitiated).async {
            background?()
            if let completion = completion {
                DispatchQueue.main.asyncAfter(deadline: .now() + delay, execute: {
                    completion()
                })
            }
        }
    }
}

extension ViewController: VerticalScrollerViewDelegate {
    func verticalScrollerView(_ horizontalScrollerView: VerticalScrollerView, didSelectViewAt index: Int) {
    //1
    let previousDatabaseView = horizontalScrollerView.view(at: currentDatabaseIndex) as! DatabaseView
    previousDatabaseView.highlightDatabase(false)
    //2
    currentDatabaseIndex = index
    //3
    let databaseView = horizontalScrollerView.view(at: currentDatabaseIndex) as! DatabaseView
    databaseView.highlightDatabase(true)
    //4
  }
}

extension ViewController: VerticalViewDataSource {
  func numberOfViews(in horizontalScrollerView: VerticalScrollerView) -> Int {
    return databases.count
  }
  
  func getScrollerViewItem(_ horizontalScrollerView: VerticalScrollerView, viewAt index: Int) -> UIView {
    print(databases[index].path)
    let databaseView = DatabaseView(frame: CGRect(x: 0, y: 0, width: 100, height: 100), databaseURL: databases[index])

    databaseView.delegate = self
    
    if currentDatabaseIndex == index {
        databaseView.highlightDatabase(true)
    } else {
        databaseView.highlightDatabase(false)
    }

    return databaseView
  }
}

extension ViewController: DatabaseViewDelegate {
    func databaseShared(databaseURL: URL) {
        self.dismiss(animated: true)
        self.shareFile(databaseURL)
    }
    
    func databaseRenamed(databaseURL: URL) {
        self.dismiss(animated: true)
        
        if(openedDatabasePath?.lastPathComponent == databaseURL.lastPathComponent)
        {
            let alertController = UIAlertController(title: "Rename Database", message: "Database \(databaseURL.lastPathComponent) is already opened, cannot rename it.", preferredStyle: .alert)
            let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            }
            alertController.addAction(okAction)
            present(alertController, animated: true)
            return
        }
        
        self.rename(fileURL: databaseURL)
    }
    
    func databaseDeleted(databaseURL: URL) {
        self.dismiss(animated: true)
        
        if(openedDatabasePath?.lastPathComponent == databaseURL.lastPathComponent)
        {
            let alertController = UIAlertController(title: "Delete Database", message: "Database \(databaseURL.lastPathComponent) is already opened, cannot delete it.", preferredStyle: .alert)
            let okAction = UIAlertAction(title: "OK", style: .default) { (action) in
            }
            alertController.addAction(okAction)
            present(alertController, animated: true)
            return
        }
        
        do {
            try FileManager.default.removeItem(at: databaseURL)
            print("File \(databaseURL) deleted")
        }
        catch {
            print("Error deleting file \(databaseURL)")
        }
        self.updateDatabases()
        if(!databases.isEmpty)
        {
            self.openLibrary()
        }
        else {
            self.updateState(state: self.mState)
        }
    }
  }

extension UserDefaults {
    func reset() {
        let defaults = UserDefaults.standard
        defaults.dictionaryRepresentation().keys.forEach(defaults.removeObject(forKey:))
        
        setDefaultsFromSettingsBundle()
    }
}

extension SKStoreReviewController {
    public static func requestReviewInCurrentScene() {
        if let scene = UIApplication.shared.connectedScenes.first(where: { $0.activationState == .foregroundActive }) as? UIWindowScene {
            requestReview(in: scene)
        }
    }
}
