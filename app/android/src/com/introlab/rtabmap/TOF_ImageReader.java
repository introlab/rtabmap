package com.introlab.rtabmap;


import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.util.Log;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TOF_ImageReader implements ImageReader.OnImageAvailableListener {

	public static final String TAG = TOF_ImageReader.class.getSimpleName();
	
	public int WIDTH;
	public int HEIGHT;
	
    public ImageReader imageReader;
    public int frameCount = 0;
    public long timestamp;
    
    // Looper handler thread.
    private HandlerThread backgroundThread;
    // Looper handler.
    public Handler backgroundHandler;

    public ByteBuffer depth16_raw;
    
    // Camera capture session. 
 	private CameraCaptureSession captureSession = null;
 	// Camera device. 
 	private CameraDevice cameraDevice = null;
 	
 // Camera preview capture request builder
 	private CaptureRequest.Builder previewCaptureRequestBuilder;


    TOF_ImageReader(){
    }
    
    public void close()
    {
    	Log.i(TAG, "close()");
    	if (captureSession != null) {
			captureSession.close();
			captureSession = null;
		}
		if (cameraDevice != null) {
			cameraDevice.close();
			cameraDevice = null;
		}
		
		if (imageReader != null) {
			imageReader.close();
			imageReader = null;
		}
    }
    
 // Camera device state callback.
 	public final CameraDevice.StateCallback cameraDeviceCallback =
 			new CameraDevice.StateCallback() {
 		@Override
 		public void onOpened(@NonNull CameraDevice cameraDevice) {
 			Log.d(TAG, "Camera depth ID " + cameraDevice.getId() + " opened.");
 			TOF_ImageReader.this.cameraDevice = cameraDevice;
 			createCameraPreviewSession();
 		}

 		@Override
 		public void onClosed(@NonNull CameraDevice cameraDevice) {
 			Log.d(TAG, "Camera device ID " + cameraDevice.getId() + " closed.");
 			TOF_ImageReader.this.cameraDevice = null;
 		}

 		@Override
 		public void onDisconnected(@NonNull CameraDevice cameraDevice) {
 			Log.w(TAG, "Camera depth ID " + cameraDevice.getId() + " disconnected.");
 			cameraDevice.close();
 			TOF_ImageReader.this.cameraDevice = null;
 		}

 		@Override
 		public void onError(@NonNull CameraDevice cameraDevice, int error) {
 			Log.e(TAG, "Camera depth ID " + cameraDevice.getId() + " error " + error);
 			cameraDevice.close();
 			TOF_ImageReader.this.cameraDevice = null;
 		}
 	};
 	
 	private CameraCaptureSession.StateCallback captureStateCallback = new CameraCaptureSession.StateCallback() {
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
		frameCount = 0;
		try {
			
			// Create an ARCore compatible capture request using `TEMPLATE_RECORD`.
			previewCaptureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);

			previewCaptureRequestBuilder.addTarget(imageReader.getSurface());

			// Create camera capture session for camera preview using callback.
			cameraDevice.createCaptureSession(Arrays.asList(imageReader.getSurface()), captureStateCallback, backgroundHandler);

		} catch (CameraAccessException e) {
			Log.e(TAG, "CameraAccessException", e);
		}
	}

    public void createImageReader(int width, int height){
    	Log.w(TAG, String.valueOf(width) + "x" + String.valueOf(height));
    	WIDTH = width;
    	HEIGHT = height;
        this.imageReader =
                ImageReader.newInstance(
                        width,
                        height,
                        ImageFormat.DEPTH16,
                        2);
        this.imageReader.setOnImageAvailableListener(this, this.backgroundHandler);
    }

    // CPU image reader callback.
    @Override
    public void onImageAvailable(ImageReader imageReader) {
        Image image  = imageReader.acquireLatestImage();
        if (image == null) {
            Log.w(TAG, "onImageAvailable: Skipping null image.");
            return;
        }
        else{
            if(image.getFormat() == ImageFormat.DEPTH16){
                synchronized (this) {
                	 depth16_raw = image.getPlanes()[0].getBuffer().asReadOnlyBuffer();
                	 timestamp = image.getTimestamp();
				}
                // copy raw undecoded DEPTH16 format depth data to NativeBuffer
                frameCount++;
            }
            else{
                Log.w(TAG, "onImageAvailable: depth image not in DEPTH16 format, skipping image");
            }
        }
        image.close();
    }
    
 // Start background handler thread, used to run callbacks without blocking UI thread.
    public void startBackgroundThread() {
        this.backgroundThread = new HandlerThread("DepthDecoderThread");
        this.backgroundThread.start();
        this.backgroundHandler = new Handler(backgroundThread.getLooper());
    }

    // Stop background handler thread.
    public void stopBackgroundThread() {
        if (this.backgroundThread != null) {
            this.backgroundThread.quitSafely();
            try {
                this.backgroundThread.join();
                this.backgroundThread = null;
                this.backgroundHandler = null;
            } catch (InterruptedException e) {
                Log.e(TAG, "Interrupted while trying to join depth background handler thread", e);
            }
        }
    }

}

