package com.introlab.rtabmap;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.google.ar.core.Anchor;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraConfig;
import com.google.ar.core.CameraConfigFilter;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Config;
import com.google.ar.core.Coordinates2d;
import com.google.ar.core.Frame;
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
import android.hardware.camera2.CameraMetadata;
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
import android.widget.Toast;

public class ARCoreSharedCamera {

	public static final String TAG = ARCoreSharedCamera.class.getSimpleName();

	private static final float[] QUAD_COORDS =
      new float[] {
        -1.0f, -1.0f, +1.0f, -1.0f, -1.0f, +1.0f, +1.0f, +1.0f,
      };
	
	private static RTABMapActivity mActivity;
	public ARCoreSharedCamera(RTABMapActivity c, float arCoreLocalizationFilteringSpeed) {
		mActivity = c;
		mARCoreLocalizationFilteringSpeed = arCoreLocalizationFilteringSpeed;
	}

	// Depth TOF Image.
	// Use 240 * 180 for now, hardcoded for Huawei P30 Pro
	private int depthWidth = 640;
	private int depthHeight = 480;

	// GL Surface used to draw camera preview image.
	public GLSurfaceView surfaceView;

	// ARCore session that supports camera sharing.
	private Session sharedSession;
	
	private Pose previousAnchorPose = null;
	private long previousAnchorTimeStamp;
	private Pose arCoreCorrection = Pose.IDENTITY;
	private Pose odomPose = Pose.IDENTITY;
	private float mARCoreLocalizationFilteringSpeed = 1.0f;

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
	
	private Toast mToast = null;

	// Camera ID for the camera used by ARCore.
	private String cameraId;
	private String depthCameraId;
	
	private Pose rgbExtrinsics;
	private Pose depthExtrinsics;
	private float[] depthIntrinsics = null;
	
	private AtomicBoolean mReady = new AtomicBoolean(false);

	// Camera preview capture request builder
	private CaptureRequest.Builder previewCaptureRequestBuilder;

	private int cameraTextureId = -1;

	// Image reader that continuously processes CPU images.
	public TOF_ImageReader mTOFImageReader = new TOF_ImageReader();
	private boolean mTOFAvailable = false;
	
	public boolean isDepthSupported() {return mTOFAvailable;}

	public void setToast(Toast toast)
	{
		mToast = toast;
	}
	
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
			//Log.i(TAG, "onCaptureCompleted");
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
			if (mTOFAvailable && cameraId.compareTo(depthCameraId) == 0) surfaceList.add(mTOFImageReader.imageReader.getSurface());
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

    /**
     * Return true if the given array contains the given integer.
     *
     * @param modes array to check.
     * @param mode  integer to get for.
     * @return true if the array contains the given integer, otherwise false.
     */
    private static boolean contains(int[] modes, int mode) {
        if (modes == null) {
            return false;
        }
        for (int i : modes) {
            if (i == mode) {
                return true;
            }
        }
        return false;
    }
    
	// Perform various checks, then open camera device and create CPU image reader.
	public boolean openCamera() {

		close();
		
		startBackgroundThread();

		if(cameraTextureId == -1)
		{
			int[] textures = new int[1];
			GLES20.glGenTextures(1, textures, 0);
			cameraTextureId = textures[0];
		}

		Log.v(TAG, "Perform various checks, then open camera device and create CPU image reader.");
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
			
			// First obtain the session handle before getting the list of various camera configs.
			// Create filter here with desired fps filters.
			CameraConfigFilter cameraConfigFilter = new CameraConfigFilter(sharedSession);
			CameraConfig[] cameraConfigs = sharedSession.getSupportedCameraConfigs(cameraConfigFilter).toArray(new CameraConfig[0]);
			Log.i(TAG, "Size of supported CameraConfigs list is " + cameraConfigs.length);

			// Determine the highest and lowest CPU resolutions.
			int highestResolutionIndex=-1;
			int highestResolution = 0;
			for(int i=0; i<cameraConfigs.length; ++i)
			{
				Log.i(TAG, "Camera ID: " + cameraConfigs[i].getCameraId());
				Log.i(TAG, "Resolution: " + cameraConfigs[i].getImageSize().getWidth() + "x" + cameraConfigs[i].getImageSize().getHeight());
				if(highestResolution == 0 || highestResolution < cameraConfigs[i].getImageSize().getWidth())
				{
					highestResolutionIndex = i;
					highestResolution = cameraConfigs[i].getImageSize().getWidth();
				}
			}
			if(highestResolutionIndex>=0)
			{
				//Log.i(TAG, "Setting camera resolution to " + cameraConfigs[highestResolutionIndex].getImageSize().getWidth() + "x" + cameraConfigs[highestResolutionIndex].getImageSize().getHeight());
				//FIXME: Can we make it work to use HD rgb images? To avoid this error "CaptureRequest contains unconfigured Input/Output Surface!"
				//sharedSession.setCameraConfig(cameraConfigs[highestResolutionIndex]);
			}

			// Enable auto focus mode while ARCore is running.
			Config config = sharedSession.getConfig();
			config.setFocusMode(Config.FocusMode.FIXED);
			config.setUpdateMode(Config.UpdateMode.BLOCKING);
			config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED);
			config.setLightEstimationMode(Config.LightEstimationMode.DISABLED);
			config.setCloudAnchorMode(Config.CloudAnchorMode.DISABLED);
			sharedSession.configure(config);

		}

		// Store the ARCore shared camera reference.
		sharedCamera = sharedSession.getSharedCamera();
		// Store the ID of the camera used by ARCore.
		cameraId = sharedSession.getCameraConfig().getCameraId();

		Log.d(TAG, "Shared camera ID: " + cameraId);

		mTOFAvailable = false;

		// Store a reference to the camera system service.
		cameraManager = (CameraManager) mActivity.getSystemService(Context.CAMERA_SERVICE);
		
		depthCameraId = null;
		// show all cameras
		try {
            // Find a CameraDevice that supports DEPTH16 captures, and configure state.
            for (String tmpCameraId : cameraManager.getCameraIdList()) {
                CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(tmpCameraId);

                Log.i(TAG, "Camera " + tmpCameraId + " extrinsics:");
                
            	float[] translation = characteristics.get(CameraCharacteristics.LENS_POSE_TRANSLATION);
                if(translation != null)
                {
                	 Log.i(TAG, String.format("Translation (x,y,z): %f,%f,%f", translation[0], translation[1], translation[2]));
                }
                float[] rotation = characteristics.get(CameraCharacteristics.LENS_POSE_ROTATION);
                if(rotation != null)
                {
                	 Log.i(TAG, String.format("Rotation (qx,qy,qz,qw): %f,%f,%f,%f",  rotation[0], rotation[1], rotation[2], rotation[3]));
                }
                
                if(tmpCameraId.compareTo(cameraId)==0 && translation!=null && rotation!=null)
                {
                	rgbExtrinsics = new Pose(translation, rotation);
                	Log.i(TAG,"Set rgb extrinsics!");
                }
                
                if (!contains(characteristics.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES), CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT) ||
                		characteristics.get(CameraCharacteristics.LENS_FACING) == CameraMetadata.LENS_FACING_FRONT) {
                    continue;
                }
                Log.i(TAG, "Camera " + tmpCameraId + " has depth output available");
                
                depthCameraId = tmpCameraId;
                if(translation!=null && rotation != null)
                {
                	depthExtrinsics = new Pose(translation, rotation);
                	Log.i(TAG,"Set depth extrinsics!");
                }
                depthIntrinsics = characteristics.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION);
                Log.i(TAG, String.format("Intrinsics (fx,fy,cx,cy,s): %f,%f,%f,%f,%f", 
                		depthIntrinsics[0],
                		depthIntrinsics[1],
                		depthIntrinsics[2],
                		depthIntrinsics[3],
                		depthIntrinsics[4]));
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
		if(rgbExtrinsics == null)
		{
			float[] translation = {0,0,0};
			float[] rotation = {0,0,0,1};
			rgbExtrinsics = new Pose(translation, rotation);
		}
		if(depthExtrinsics == null)
		{
			depthExtrinsics = rgbExtrinsics;
		}

		if(depthCameraId != null)
		{
			ArrayList<String> resolutions = getResolutions(mActivity, depthCameraId, ImageFormat.DEPTH16);
			if (resolutions != null) {
				float[] newDepthIntrinsics = null;
				int largestWidth = 0;
				for( String temp : resolutions) {
					Log.i(TAG, "DEPTH16 resolution: " + temp);
					depthWidth = Integer.parseInt(temp.split("x")[0]);
					depthHeight = Integer.parseInt(temp.split("x")[1]);
					if(depthIntrinsics != null)
					{
						if(depthIntrinsics[0] != 0)
						{
							if(largestWidth == 0 && depthWidth>largestWidth)
							{
								largestWidth = depthWidth; // intrinsics should match this resolution
							}
							// Samsung Galaxy Note10+: take smallest resolution and match the intrinsics
							if(depthWidth < largestWidth)
							{
								float scale = (float)depthWidth/(float)largestWidth;
								newDepthIntrinsics = depthIntrinsics.clone();
								newDepthIntrinsics[0] *= scale;
								newDepthIntrinsics[1] *= scale;
								newDepthIntrinsics[2] *= scale;
								newDepthIntrinsics[3] *= scale;
							}
						}
						else if(depthWidth ==240 && depthHeight==180)
						{
							// Huawei P30 Pro: only 240x180 is working
							break;
						}
					}
				}
				if (resolutions.size()>0) {
					mTOFAvailable = true;
					if(newDepthIntrinsics!=null) {
						depthIntrinsics = newDepthIntrinsics;
					}
				}
			}
		}
		Log.i(TAG, "TOF_available: " + mTOFAvailable);

		// Color CPU Image.
		// Use the currently configured CPU image size.
		//Size desiredCPUImageSize = sharedSession.getCameraConfig().getImageSize();

		if (mTOFAvailable) mTOFImageReader.createImageReader(depthWidth, depthHeight);

		// When ARCore is running, make sure it also updates our CPU image surface.
		if (mTOFAvailable && cameraId.compareTo(depthCameraId) == 0) {
			sharedCamera.setAppSurfaces(this.cameraId, Arrays.asList(mTOFImageReader.imageReader.getSurface()));
		}

		try {

			// Wrap our callback in a shared camera callback.
			CameraDevice.StateCallback wrappedCallback = sharedCamera.createARDeviceStateCallback(cameraDeviceCallback, backgroundHandler);

			// Open the camera device using the ARCore wrapped callback.
			cameraManager.openCamera(cameraId, wrappedCallback, backgroundHandler);
			
			if(mTOFAvailable && cameraId.compareTo(depthCameraId) != 0)
			{
				cameraManager.openCamera(depthCameraId, mTOFImageReader.cameraDeviceCallback, mTOFImageReader.backgroundHandler);
			}
			
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
		
		return true;
	}
	
	public static void rotationMatrixToQuaternion(float[] R, float[] q) {
        final float m00 = R[0];
        final float m10 = R[1];
        final float m20 = R[2];
        final float m01 = R[4];
        final float m11 = R[5];
        final float m21 = R[6];
        final float m02 = R[8];
        final float m12 = R[9];
        final float m22 = R[10];

        float tr = m00 + m11 + m22;

        if (tr > 0) {
            float S = (float) Math.sqrt(tr + 1.0) * 2; // S=4*qw 
            q[0] = 0.25f * S;/* w  w  w.j ava  2s.co m*/
            q[1] = (m21 - m12) / S;
            q[2] = (m02 - m20) / S;
            q[3] = (m10 - m01) / S;
        } else if ((m00 > m11) & (m00 > m22)) {
            float S = (float) Math.sqrt(1.0 + m00 - m11 - m22) * 2; //
            // S=4*q[1]
            q[0] = (m21 - m12) / S;
            q[1] = 0.25f * S;
            q[2] = (m01 + m10) / S;
            q[3] = (m02 + m20) / S;
        } else if (m11 > m22) {
            float S = (float) Math.sqrt(1.0 + m11 - m00 - m22) * 2; //
            // S=4*q[2]
            q[0] = (m02 - m20) / S;
            q[1] = (m01 + m10) / S;
            q[2] = 0.25f * S;
            q[3] = (m12 + m21) / S;
        } else {
            float S = (float) Math.sqrt(1.0 + m22 - m00 - m11) * 2; //
            // S=4*q[3]
            q[0] = (m10 - m01) / S;
            q[1] = (m02 + m20) / S;
            q[2] = (m12 + m21) / S;
            q[3] = 0.25f * S;
        }

    }


	// Close the camera device.
	public void close() {
		Log.w(TAG, "close()");
		
		if (sharedSession != null)  {
			sharedSession.close();
			sharedSession = null;
		}

		if (captureSession != null) {
			captureSession.close();
			captureSession = null;
		}
		if (cameraDevice != null) {
			cameraDevice.close();
			cameraDevice = null;
		}
		
		mTOFImageReader.close();

		if(cameraTextureId>=0)
		{
			GLES20.glDeleteTextures(1, new int[] {cameraTextureId}, 0);
		}
		
		stopBackgroundThread();
	}

	public void setDisplayGeometry(int rotation, int width, int height)
	{
		if(sharedSession!=null)
			sharedSession.setDisplayGeometry(rotation, width, height);
	}
	
	/*************************************************** ONDRAWFRAME ARCORE ************************************************************* */	
	// Draw frame when in AR mode. Called on the GL thread.
	public void updateGL() throws CameraNotAvailableException {

		if(!mReady.get() || sharedSession == null)
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
			Log.e(TAG, String.format("frame.getCamera() null!"));
			return;
		}

		if (camera == null) { 
			Log.e(TAG, String.format("camera is null!"));
			return;
		}
		if (camera.getTrackingState() != TrackingState.TRACKING) {
			final String trackingState = camera.getTrackingState().toString();
			Log.e(TAG, String.format("Tracking lost! state=%s", trackingState));
			// This will force a new session on the next frame received
			RTABMapLib.postCameraPoseEvent(RTABMapActivity.nativeApplication, 0,0,0,0,0,0,0,0);
			mActivity.runOnUiThread(new Runnable() {
				public void run() {
					if(mToast!=null && previousAnchorPose != null)
					{
						String msg = "Tracking lost! If you are mapping, you will need to relocalize before continuing.";
						if(mToast.getView() == null || !mToast.getView().isShown())
						{
							mToast.makeText(mActivity.getApplicationContext(), 
									msg, Toast.LENGTH_LONG).show();
						}
						else
						{
							mToast.setText(msg);
						}
						previousAnchorPose = null;
					}
				}
			});
			return;
		}

		if (frame.getTimestamp() != 0) {
			Pose pose = camera.getPose();
			
			// Remove ARCore SLAM corrections by integrating pose from previous frame anchor
			if(previousAnchorPose == null || mARCoreLocalizationFilteringSpeed==0)
			{
				odomPose = pose;
			}
			else
			{
				float[] t = previousAnchorPose.inverse().compose(pose).getTranslation();
				final double speed = Math.sqrt(t[0]*t[0]+t[1]*t[1]+t[2]*t[2])/((double)(frame.getTimestamp()-previousAnchorTimeStamp)/10e8);
				if(speed>=mARCoreLocalizationFilteringSpeed)
				{
					arCoreCorrection = arCoreCorrection.compose(previousAnchorPose).compose(pose.inverse());
					t = arCoreCorrection.getTranslation();
					Log.e(TAG, String.format("POTENTIAL TELEPORTATION!!!!!!!!!!!!!! previous anchor moved (speed=%f), new arcorrection: %f %f %f", speed, t[0], t[1], t[2]));
					
					t = odomPose.getTranslation();
					float[] t2 = (arCoreCorrection.compose(pose)).getTranslation();
					float[] t3 = previousAnchorPose.getTranslation();
					float[] t4 = pose.getTranslation();
					Log.e(TAG, String.format("Odom = %f %f %f -> %f %f %f ArCore= %f %f %f -> %f %f %f", t[0], t[1], t[2], t2[0], t2[1], t2[2], t3[0], t3[1], t3[2], t4[0], t4[1], t4[2]));
				
					mActivity.runOnUiThread(new Runnable() {
						public void run() {
							if(mToast!=null)
							{
								String msg = String.format("ARCore localization has been suppressed "
										+ "because of high speed detected (%f m/s) causing a jump! You can change "
										+ "ARCore localization filtering speed in Settings->Mapping if you are "
										+ "indeed moving as fast.", speed);
								if(mToast.getView() == null || !mToast.getView().isShown())
								{
									mToast.makeText(mActivity.getApplicationContext(), msg, Toast.LENGTH_LONG).show();
								}
								else
								{
									mToast.setText(msg);
								}
								previousAnchorPose = null;
							}
						}
					});
				}
				odomPose = arCoreCorrection.compose(pose);
			}
			previousAnchorPose = pose;
			previousAnchorTimeStamp = frame.getTimestamp();
			
			double stamp = (double)frame.getTimestamp()/10e8;
			if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("pose=%f %f %f q=%f %f %f %f stamp=%f", odomPose.tx(), odomPose.ty(), odomPose.tz(), odomPose.qx(), odomPose.qy(), odomPose.qz(), odomPose.qw(), stamp));
			RTABMapLib.postCameraPoseEvent(RTABMapActivity.nativeApplication, odomPose.tx(), odomPose.ty(), odomPose.tz(), odomPose.qx(), odomPose.qy(), odomPose.qz(), odomPose.qw(), stamp);
			
			CameraIntrinsics intrinsics = camera.getImageIntrinsics();
			try{
				Image image = frame.acquireCameraImage();
				if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("frame=%d vs image=%d", frame.getTimestamp(), image.getTimestamp()));
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

				if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("RGB %dx%d len=%dbytes format=%d stamp=%f", 
						image.getWidth(), image.getHeight(), y.limit(), image.getFormat(), stamp));
				
				float[] texCoord = new float[8];
				frame.transformCoordinates2d(
				        Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES,
				        QUAD_COORDS,
				        Coordinates2d.IMAGE_NORMALIZED,
				        texCoord);
				
				float[] p = new float[16];
		        camera.getProjectionMatrix(p, 0, 0.1f, 100.0f);
   
		        float[] viewMatrix = new float[16];
		        arCoreCorrection.compose(camera.getDisplayOrientedPose()).inverse().toMatrix(viewMatrix, 0);
		        float[] quat = new float[4];
		        rotationMatrixToQuaternion(viewMatrix, quat);

		        
				if(mTOFAvailable)
				{
					ByteBuffer depth;
					double depthStamp;
					synchronized (mTOFImageReader) {
						depth = mTOFImageReader.depth16_raw;
						depthStamp = (double)mTOFImageReader.timestamp/10e8;
					}
					
					if(!RTABMapActivity.DISABLE_LOG) Log.d(TAG, String.format("Depth %dx%d len=%dbytes format=%d stamp=%f",
							mTOFImageReader.WIDTH, mTOFImageReader.HEIGHT, depth.limit(), ImageFormat.DEPTH16, depthStamp));

					RTABMapLib.postOdometryEventDepth(
							RTABMapActivity.nativeApplication,
							odomPose.tx(), odomPose.ty(), odomPose.tz(), odomPose.qx(), odomPose.qy(), odomPose.qz(), odomPose.qw(), 
							fl[0], fl[1], pp[0], pp[1], 
							depthIntrinsics[0], depthIntrinsics[1], depthIntrinsics[2], depthIntrinsics[3],
							rgbExtrinsics.tx(), rgbExtrinsics.ty(), rgbExtrinsics.tz(), rgbExtrinsics.qx(), rgbExtrinsics.qy(), rgbExtrinsics.qz(), rgbExtrinsics.qw(),
							depthExtrinsics.tx(), depthExtrinsics.ty(), depthExtrinsics.tz(), depthExtrinsics.qx(), depthExtrinsics.qy(), depthExtrinsics.qz(), depthExtrinsics.qw(),
							stamp,
							depthStamp,
							y, u, v, y.limit(), image.getWidth(), image.getHeight(), image.getFormat(), 
							depth, depth.limit(), mTOFImageReader.WIDTH, mTOFImageReader.HEIGHT, ImageFormat.DEPTH16,
							points, points.limit()/4,
							viewMatrix[12], viewMatrix[13], viewMatrix[14], quat[1], quat[2], quat[3], quat[0],
							p[0], p[5], p[8], p[9], p[10], p[11], p[14],
                            texCoord[0],texCoord[1],texCoord[2],texCoord[3],texCoord[4],texCoord[5],texCoord[6],texCoord[7]);
				}
				else
				{
					RTABMapLib.postOdometryEvent(
							RTABMapActivity.nativeApplication,
							odomPose.tx(), odomPose.ty(), odomPose.tz(), odomPose.qx(), odomPose.qy(), odomPose.qz(), odomPose.qw(), 
							fl[0], fl[1], pp[0], pp[1], 
							rgbExtrinsics.tx(), rgbExtrinsics.ty(), rgbExtrinsics.tz(), rgbExtrinsics.qx(), rgbExtrinsics.qy(), rgbExtrinsics.qz(), rgbExtrinsics.qw(),
							stamp, 
							y, u, v, y.limit(), image.getWidth(), image.getHeight(), image.getFormat(), 
							points, points.limit()/4,
							viewMatrix[12], viewMatrix[13], viewMatrix[14], quat[1], quat[2], quat[3], quat[0],
							p[0], p[5], p[8], p[9], p[10], p[11], p[14],
                            texCoord[0],texCoord[1],texCoord[2],texCoord[3],texCoord[4],texCoord[5],texCoord[6],texCoord[7]);
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


	public ArrayList<String> getResolutions (Context context, String cameraId, int imageFormat){
		Log.v(TAG, "getResolutions: cameraId:" + cameraId + " imageFormat: " + imageFormat);

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

}
