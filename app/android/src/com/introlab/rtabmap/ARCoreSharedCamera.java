package com.introlab.rtabmap;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.ar.core.Camera;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.ImageMetadata;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.SharedCamera;
import com.google.ar.core.TrackingState;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.core.exceptions.UnavailableException;

import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.util.Log;
import android.view.Surface;

public class ARCoreSharedCamera {

	public static final String TAG = ARCoreSharedCamera.class.getSimpleName();


	private static RTABMapActivity mActivity;
	public ARCoreSharedCamera(RTABMapActivity c) {
		mActivity = c;
	}

	// Depth TOF Image.
	// Use 240 * 180 for now, hardcoded for Huawei P30 Pro
	private static final int DEPTH_WIDTH = 240;
	private static final int DEPTH_HEIGHT = 180;

	// GL Surface used to draw camera preview image.
	public GLSurfaceView surfaceView;

	// ARCore session that supports camera sharing.
	private Session sharedSession;

	// Camera capture session. Used by both non-AR and AR modes.
	private CameraCaptureSession captureSession;

	// Reference to the camera system service.
	private CameraManager cameraManager;

	// Camera device. Used by both non-AR and AR modes.
	private CameraDevice cameraDevice;

	// Looper handler thread.
	private HandlerThread backgroundThread;
	// Looper handler.
	private Handler backgroundHandler;

	// ARCore shared camera instance, obtained from ARCore session that supports sharing.
	private SharedCamera sharedCamera;

	// Camera ID for the camera used by ARCore.
	private String cameraId;
	
	private AtomicBoolean mReady = new AtomicBoolean(false);

	// Camera preview capture request builder
	private CaptureRequest.Builder previewCaptureRequestBuilder;

	private int cameraTextureId = -1;

	// Image reader that continuously processes CPU images.
	public TOF_ImageReader mTOFImageReader = new TOF_ImageReader();
	private boolean mTOFAvailable = false;
	
	public boolean isDepthSupported() {return mTOFAvailable;}

	// Camera device state callback.
	private final CameraDevice.StateCallback cameraDeviceCallback =
			new CameraDevice.StateCallback() {
		@Override
		public void onOpened(@NonNull CameraDevice cameraDevice) {
			Log.d(TAG, "Camera device ID " + cameraDevice.getId() + " opened.");
			ARCoreSharedCamera.this.cameraDevice = cameraDevice;
			createCameraPreviewSession();
		}

		@Override
		public void onClosed(@NonNull CameraDevice cameraDevice) {
			Log.d(TAG, "Camera device ID " + cameraDevice.getId() + " closed.");
			ARCoreSharedCamera.this.cameraDevice = null;
		}

		@Override
		public void onDisconnected(@NonNull CameraDevice cameraDevice) {
			Log.w(TAG, "Camera device ID " + cameraDevice.getId() + " disconnected.");
			cameraDevice.close();
			ARCoreSharedCamera.this.cameraDevice = null;
		}

		@Override
		public void onError(@NonNull CameraDevice cameraDevice, int error) {
			Log.e(TAG, "Camera device ID " + cameraDevice.getId() + " error " + error);
			cameraDevice.close();
			ARCoreSharedCamera.this.cameraDevice = null;
		}
	};

	// Repeating camera capture session state callback.
	CameraCaptureSession.StateCallback cameraCaptureCallback =
			new CameraCaptureSession.StateCallback() {

		// Called when the camera capture session is first configured after the app
		// is initialized, and again each time the activity is resumed.
		@Override
		public void onConfigured(@NonNull CameraCaptureSession session) {
			Log.d(TAG, "Camera capture session configured.");
			captureSession = session;
			setRepeatingCaptureRequest();
		}

		@Override
		public void onSurfacePrepared(
				@NonNull CameraCaptureSession session, @NonNull Surface surface) {
			Log.d(TAG, "Camera capture surface prepared.");
		}

		@Override
		public void onReady(@NonNull CameraCaptureSession session) {
			Log.d(TAG, "Camera capture session ready.");
		}

		@Override
		public void onActive(@NonNull CameraCaptureSession session) {
			Log.d(TAG, "Camera capture session active.");
			resumeARCore();
		}

		@Override
		public void onClosed(@NonNull CameraCaptureSession session) {
			Log.d(TAG, "Camera capture session closed.");
		}

		@Override
		public void onConfigureFailed(@NonNull CameraCaptureSession session) {
			Log.e(TAG, "Failed to configure camera capture session.");
		}
	};

	// Repeating camera capture session capture callback.
	private final CameraCaptureSession.CaptureCallback captureSessionCallback =
			new CameraCaptureSession.CaptureCallback() {

		@Override
		public void onCaptureCompleted(
				@NonNull CameraCaptureSession session,
				@NonNull CaptureRequest request,
				@NonNull TotalCaptureResult result) {
			Log.i(TAG, "onCaptureCompleted");
		}

		//@Override // android 23 
		public void onCaptureBufferLost(
				@NonNull CameraCaptureSession session,
				@NonNull CaptureRequest request,
				@NonNull Surface target,
				long frameNumber) {
			Log.e(TAG, "onCaptureBufferLost: " + frameNumber);
		}

		@Override
		public void onCaptureFailed(
				@NonNull CameraCaptureSession session,
				@NonNull CaptureRequest request,
				@NonNull CaptureFailure failure) {
			Log.e(TAG, "onCaptureFailed: " + failure.getFrameNumber() + " " + failure.getReason());
		}

		@Override
		public void onCaptureSequenceAborted(
				@NonNull CameraCaptureSession session, int sequenceId) {
			Log.e(TAG, "onCaptureSequenceAborted: " + sequenceId + " " + session);
		}
	};

	private void resumeARCore() {
		// Ensure that session is valid before triggering ARCore resume. Handles the case where the user
		// manually uninstalls ARCore while the app is paused and then resumes.
		if (sharedSession == null) {
			return;
		}

		try {
			Log.i(TAG, "Resume ARCore.");
			// Resume ARCore.
			sharedSession.resume();
			// Set capture session callback while in AR mode.
			sharedCamera.setCaptureCallback(captureSessionCallback, backgroundHandler);
		} catch (CameraNotAvailableException e) {
			Log.e(TAG, "Failed to resume ARCore session", e);
			return;
		}

	}


	// Called when starting non-AR mode or switching to non-AR mode.
	// Also called when app starts in AR mode, or resumes in AR mode.
	private void setRepeatingCaptureRequest() {
		try {
			captureSession.setRepeatingRequest(
					previewCaptureRequestBuilder.build(), captureSessionCallback, backgroundHandler);
		} catch (CameraAccessException e) {
			Log.e(TAG, "Failed to set repeating request", e);
		}
	}


	private void createCameraPreviewSession() {
		Log.e(TAG, "createCameraPreviewSession: " + "starting camera preview session.");
		try {
			// Note that isGlAttached will be set to true in AR mode in onDrawFrame().
			sharedSession.setCameraTextureName(cameraTextureId);

			// Create an ARCore compatible capture request using `TEMPLATE_RECORD`.
			previewCaptureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);

			// Build surfaces list, starting with ARCore provided surfaces.
			List<Surface> surfaceList = sharedCamera.getArCoreSurfaces();
			Log.e(TAG, " createCameraPreviewSession: " + "surfaceList: sharedCamera.getArCoreSurfaces(): " + surfaceList.size());

			// Add a CPU image reader surface. On devices that don't support CPU image access, the image
			// may arrive significantly later, or not arrive at all.
			if (mTOFAvailable) surfaceList.add(mTOFImageReader.imageReader.getSurface());
			// Surface list should now contain three surfacemReadymReadys:
			// 0. sharedCamera.getSurfaceTexture()
			// 1. …
			// 2. depthImageReader.getSurface()

			// Add ARCore surfaces and CPU image surface targets.
			for (Surface surface : surfaceList) {
				previewCaptureRequestBuilder.addTarget(surface);
			}

			// Wrap our callback in a shared camera callback.
			CameraCaptureSession.StateCallback wrappedCallback = sharedCamera.createARSessionStateCallback(cameraCaptureCallback, backgroundHandler);

			// Create camera capture session for camera preview using ARCore wrapped callback.
			cameraDevice.createCaptureSession(surfaceList, wrappedCallback, backgroundHandler);
			
			mReady.set(true);
		} catch (CameraAccessException e) {
			Log.e(TAG, "CameraAccessException", e);
		}


	}

	// Start background handler thread, used to run callbacks without blocking UI thread.
	private void startBackgroundThread() {
		backgroundThread = new HandlerThread("sharedCameraBackground");
		backgroundThread.start();
		backgroundHandler = new Handler(backgroundThread.getLooper());
		mTOFImageReader.startBackgroundThread();
	}

	// Stop background handler thread.
	private void stopBackgroundThread() {
		if (backgroundThread != null) {
			backgroundThread.quitSafely();
			try {
				backgroundThread.join();
				backgroundThread = null;
				backgroundHandler = null;
			} catch (InterruptedException e) {
				Log.e(TAG, "Interrupted while trying to join background handler thread", e);
			}
		}
		mTOFImageReader.stopBackgroundThread();
	}
	
	private long mPreviousTime = 0;

	// Perform various checks, then open camera device and create CPU image reader.
	public boolean openCamera() {

		close();
		
		startBackgroundThread();
		
		mPreviousTime = System.currentTimeMillis();

		if(cameraTextureId == -1)
		{
			int[] textures = new int[1];
			GLES20.glGenTextures(1, textures, 0);
			cameraTextureId = textures[0];
		}

		Log.v(TAG + " opencamera: ", "Perform various checks, then open camera device and create CPU image reader.");
		// Don't open camera if already opened.
		if (cameraDevice != null) {
			return false;
		}

		if (sharedSession == null) {
			try {
				// Create ARCore session that supports camera sharing.
				sharedSession = new Session(mActivity, EnumSet.of(Session.Feature.SHARED_CAMERA));
			} catch (UnavailableException e) {
				Log.e(TAG, "Failed to create ARCore session that supports camera sharing", e);
				return false;
			}

			// Enable auto focus mode while ARCore is running.
			Config config = sharedSession.getConfig();
			config.setFocusMode(Config.FocusMode.FIXED);
			config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE);
			config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED);
			config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
			//config.setCloudAnchorMode(Config.CloudAnchorMode.ENABLED);
			sharedSession.configure(config);

		}

		// Store the ARCore shared camera reference.
		sharedCamera = sharedSession.getSharedCamera();
		// Store the ID of the camera used by ARCore.
		cameraId = sharedSession.getCameraConfig().getCameraId();
		initCamera(mActivity, cameraId, 1);
		ArrayList<String> resolutions;

		mTOFAvailable = false;

		resolutions = getResolutions(mActivity, cameraId, ImageFormat.DEPTH16);
		if (resolutions != null) {
			for( String temp : resolutions) {
				Log.e(TAG + "DEPTH16 resolution: ", temp);
			};
			if (resolutions.size()>0) mTOFAvailable = true;
		}

		// Color CPU Image.
		// Use the currently configured CPU image size.
		//Size desiredCPUImageSize = sharedSession.getCameraConfig().getImageSize();

		if (mTOFAvailable) mTOFImageReader.createImageReader(DEPTH_WIDTH, DEPTH_HEIGHT);

		// When ARCore is running, make sure it also updates our CPU image surface.
		if (mTOFAvailable) {
			sharedCamera.setAppSurfaces(this.cameraId, Arrays.asList(mTOFImageReader.imageReader.getSurface()));
		}

		try {

			// Wrap our callback in a shared camera callback.
			CameraDevice.StateCallback wrappedCallback = sharedCamera.createARDeviceStateCallback(cameraDeviceCallback, backgroundHandler);

			// Store a reference to the camera system service.
			cameraManager = (CameraManager) mActivity.getSystemService(Context.CAMERA_SERVICE);

			// Get the characteristics for the ARCore camera.
			//CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(this.cameraId);

			// Open the camera device using the ARCore wrapped callback.
			cameraManager.openCamera(cameraId, wrappedCallback, backgroundHandler);
			
		} catch (CameraAccessException e) {
			Log.e(TAG, "Failed to open camera", e);
			return false;
		} catch (IllegalArgumentException e) {
			Log.e(TAG, "Failed to open camera", e);
			return false;
		} catch (SecurityException e) {
			Log.e(TAG, "Failed to open camera", e);
			return false;
		} 

		Log.i(TAG, " opencamera: TOF_available: " + mTOFAvailable);
		return true;
	}


	// Close the camera device.
	public void close() {
		
		if (sharedSession != null)  {
			sharedSession.pause();
		}

		if (captureSession != null) {
			captureSession.close();
			captureSession = null;
		}
		if (cameraDevice != null) {
			cameraDevice.close();
		}

		if (mTOFImageReader.imageReader != null) {
			mTOFImageReader.imageReader.close();
			mTOFImageReader.imageReader = null;
		}

		if(cameraTextureId>=0)
		{
			GLES20.glDeleteTextures(1, new int[] {cameraTextureId}, 0);
		}
		
		stopBackgroundThread();
	}

	/*************************************************** ONDRAWFRAME ARCORE ************************************************************* */

	// Draw frame when in AR mode. Called on the GL thread.
	public void updateGL() throws CameraNotAvailableException {

		if(!mReady.get())
		{
			return;
		}
		
		if (mTOFAvailable && mTOFImageReader.frameCount == 0) return;
		
		// Perform ARCore per-frame update.
		Frame frame = null;
		try {
			frame = sharedSession.update();
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}

		Camera camera = null;
		if (frame != null) {
			camera = frame.getCamera();
		}else
		{
			return;
		}

		if (camera == null) return;
		// If not tracking, don't draw 3D objects.
		if (camera.getTrackingState() == TrackingState.PAUSED) return;

		if (frame.getTimestamp() != 0) {
			
			Pose pose = camera.getPose();
			if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("pose=%f %f %f q=%f %f %f %f", pose.tx(), pose.ty(), pose.tz(), pose.qx(), pose.qy(), pose.qz(), pose.qw()));
			RTABMapLib.postCameraPoseEvent(RTABMapActivity.nativeApplication, pose.tx(), pose.ty(), pose.tz(), pose.qx(), pose.qy(), pose.qz(), pose.qw());
			
			int rateMs = 100; // send images at most 10 Hz
			if(System. currentTimeMillis() - mPreviousTime < rateMs)
			{
				return;
			}
			mPreviousTime = System. currentTimeMillis();
			
			CameraIntrinsics intrinsics = camera.getImageIntrinsics();
			try{
				Image image = frame.acquireCameraImage();
				PointCloud cloud = frame.acquirePointCloud();
				FloatBuffer points = cloud.getPoints();

				if (image.getFormat() != ImageFormat.YUV_420_888) {
					throw new IllegalArgumentException(
							"Expected image in YUV_420_888 format, got format " + image.getFormat());
				}
				
				if(!RTABMapActivity.DISABLE_LOG)
				{
					for(int i =0;i<image.getPlanes().length;++i)
					{
						Log.d(TAG, String.format("Plane[%d] pixel stride = %d,  row stride = %d", i, image.getPlanes()[i].getPixelStride(), image.getPlanes()[i].getRowStride()));
					}
				}
				
				float[] fl = intrinsics.getFocalLength();
				float[] pp = intrinsics.getPrincipalPoint();
				if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("fx=%f fy=%f cx=%f cy=%f", fl[0], fl[1], pp[0], pp[1]));
				
				ByteBuffer y = image.getPlanes()[0].getBuffer().asReadOnlyBuffer();
				ByteBuffer u = image.getPlanes()[1].getBuffer().asReadOnlyBuffer();
				ByteBuffer v = image.getPlanes()[2].getBuffer().asReadOnlyBuffer();

				double stamp = (double)image.getTimestamp()/10e8;
				if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("RGB %dx%d len=%dbytes format=%d =%f",
						image.getWidth(), image.getHeight(), y.limit(), image.getFormat(), stamp));

				if(mTOFAvailable)
				{
					if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("Depth %dx%d len=%dbytes format=%d stamp=%f",
							mTOFImageReader.WIDTH, mTOFImageReader.HEIGHT, mTOFImageReader.depth16_raw.limit(), ImageFormat.DEPTH16, (double)mTOFImageReader.timestamp/10e9));
					
					RTABMapLib.postOdometryEvent(
							RTABMapActivity.nativeApplication,
							pose.tx(), pose.ty(), pose.tz(), pose.qx(), pose.qy(), pose.qz(), pose.qw(), 
							fl[0], fl[1], pp[0], pp[1], stamp, 
							y, u, v, y.limit(), image.getWidth(), image.getHeight(), image.getFormat(), 
							mTOFImageReader.depth16_raw, mTOFImageReader.depth16_raw.limit(), mTOFImageReader.WIDTH, mTOFImageReader.HEIGHT, ImageFormat.DEPTH16,
							points, points.limit()/4);
				}
				else
				{
					ByteBuffer bb = ByteBuffer.allocate(0);
					RTABMapLib.postOdometryEvent(
							RTABMapActivity.nativeApplication,
							pose.tx(), pose.ty(), pose.tz(), pose.qx(), pose.qy(), pose.qz(), pose.qw(), 
							fl[0], fl[1], pp[0], pp[1], stamp, 
							y, u, v, y.limit(), image.getWidth(), image.getHeight(), image.getFormat(), 
							bb, 0, 0, 0, ImageFormat.DEPTH16,
							points, points.limit()/4);
				}
				
				image.close();
				cloud.close();
				
			} catch (NotYetAvailableException e) {

			}
		}
	}

	/********************************************************************************************************************* */
	/*************************************************** End ************************************************************* */
	/********************************************************************************************************************* */


	public ArrayList<String> getResolutions (Context context, String cameraId,int imageFormat){
		Log.v(TAG + "getResolutions:", " cameraId:" + cameraId + " imageFormat: " + imageFormat);

		ArrayList<String> output = new ArrayList<String>();
		try {
			CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
			CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);

			for (android.util.Size s : characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(imageFormat)) {
				output.add(s.getWidth() + "x" + s.getHeight());
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		return output;
	}


	public void initCamera (Context context, String cameraId,int index){
		boolean ok = false;
		try {
			int current = 0;
			CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
			CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
			for (android.util.Size s : characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(ImageFormat.DEPTH16)) {
				ok = true;
				if (current == index)
					break;
				else ;
				current++;
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		if (!ok) {
			Log.e(TAG + " initCamera", "Depth sensor not found!");
		}
	}


}
