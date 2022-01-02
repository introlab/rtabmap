
package com.introlab.rtabmap;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;

import android.app.Activity;
import android.content.Context;
import android.os.IBinder;
import android.view.KeyEvent;
import android.util.Log;


// Wrapper for native library

public class RTABMapLib
{

	static {
        // This project depends on tango_client_api, so we need to make sure we load
        // the correct library first.
        if (TangoInitializationHelper.loadTangoSharedLibrary() ==
                TangoInitializationHelper.ARCH_ERROR) {
            Log.w(RTABMapActivity.class.getSimpleName(), "WArning! Unable to load libtango_client_api.so! This can be safely ignored if RTAB-Map NDK is not build with tango support.");
        }
        System.loadLibrary("NativeRTABMap");
    }

 // Initialize the Tango Service, this function starts the communication
    // between the application and Tango Service.
    // The activity object is used for checking if the API version is outdated.
    public static native long createNativeApplication(RTABMapActivity activity);
    
    public static native void destroyNativeApplication(long nativeApplication);
    
    public static native void setScreenRotation(long nativeApplication, int displayRotation, int cameraRotation);
    
    public static native int openDatabase(long nativeApplication, String databasePath, boolean databaseInMemory, boolean optimize, boolean clearDatabase);
    
    public static native boolean recover(long nativeApplication, String from, String to);

    public static native boolean isBuiltWith(long nativeApplication, int cameraDriver);
    public static native boolean startCamera(long nativeApplication, IBinder binder, Context context, Activity activity, int driver);
    public static native void stopCamera(long nativeApplication);

    // Allocate OpenGL resources for rendering.
    public static native void initGlContent(long nativeApplication);

    // Setup the view port width and height.
    public static native void setupGraphic(long nativeApplication, int width, int height);

    // Main render loop.
    public static native int render(long nativeApplication);

    // Set the render camera's viewing angle:
    //   first person, third person, or top down.
    public static native void setCamera(long nativeApplication, int cameraIndex);
    
    // Pass touch events to the native layer.
    public static native void onTouchEvent(long nativeApplication, int touchCount, int event0,
                                           float x0, float y0, float x1, float y1);
    
    public static native void setPausedMapping(long nativeApplication, boolean paused);
    public static native void setOnlineBlending(long nativeApplication, boolean enabled);
    public static native void setMapCloudShown(long nativeApplication, boolean shown);
    public static native void setOdomCloudShown(long nativeApplication, boolean shown);
    public static native void setMeshRendering(long nativeApplication, boolean enabled, boolean withTexture);
    public static native void setLocalizationMode(long nativeApplication, boolean enabled);
    public static native void setTrajectoryMode(long nativeApplication, boolean enabled);
    public static native void setGraphOptimization(long nativeApplication, boolean enabled);
    public static native void setNodesFiltering(long nativeApplication, boolean enabled);
    public static native void setGraphVisible(long nativeApplication, boolean visible);
    public static native void setGridVisible(long nativeApplication, boolean visible);
    public static native void setRawScanSaved(long nativeApplication, boolean enabled);
    public static native void setFullResolution(long nativeApplication, boolean enabled);
    public static native void setSmoothing(long nativeApplication, boolean enabled);
    public static native void setDepthFromMotion(long nativeApplication, boolean enabled);
    public static native void setCameraColor(long nativeApplication, boolean enabled);
    public static native void setAppendMode(long nativeApplication, boolean enabled);
    public static native void setDataRecorderMode(long nativeApplication, boolean enabled);
    public static native void setMaxCloudDepth(long nativeApplication, float value);
    public static native void setMinCloudDepth(long nativeApplication, float value);
    public static native void setPointSize(long nativeApplication, float value);
    public static native void setFOV(long nativeApplication, float value);
    public static native void setOrthoCropFactor(long nativeApplication, float value);
    public static native void setGridRotation(long nativeApplication, float value);
    public static native void setLighting(long nativeApplication, boolean enabled);
    public static native void setBackfaceCulling(long nativeApplication, boolean enabled);
    public static native void setWireframe(long nativeApplication, boolean enabled);
    public static native void setCloudDensityLevel(long nativeApplication, int value);
    public static native void setMeshAngleTolerance(long nativeApplication, float value);
    public static native void setMeshTriangleSize(long nativeApplication, int value);
    public static native void setClusterRatio(long nativeApplication, float value);
    public static native void setMaxGainRadius(long nativeApplication, float value);
    public static native void setRenderingTextureDecimation(long nativeApplication, int value);
    public static native void setBackgroundColor(long nativeApplication, float gray);
    public static native int setMappingParameter(long nativeApplication, String key, String value);
    public static native void setGPS(
    		long nativeApplication, 
			double stamp,
			double longitude, 
			double latitude, 
			double altitude,  
			double accuracy,
			double bearing);
    public static native void addEnvSensor(long nativeApplication, int type, float value);

    public static native void save(long nativeApplication, String outputDatabasePath);
    public static native void cancelProcessing(long nativeApplication);
    public static native boolean exportMesh(
    		long nativeApplication, 
    		float cloudVoxelSize,
    		boolean regenerateCloud,
    		boolean meshing,
    		int textureSize,
    		int textureCount,
    		int normalK,
    		boolean optimized,
    		float optimizedVoxelSize,
    		int optimizedDepth,
    		int optimizedMaxPolygons,
    		float optimizedColorRadius,
    		boolean optimizedCleanWhitePolygons,
    		int optimizedMinClusterSize,
    		float optimizedMaxTextureDistance,
    		int optimizedMinTextureClusterSize,
    		boolean blockRendering);
    public static native boolean writeExportedMesh(long nativeApplication, String directory, String name);
    public static native boolean postExportation(long nativeApplication, boolean visualize);
    public static native int postProcessing(long nativeApplication, int approach);
    
    public static native String getStatus(long nativeApplication);
    public static native int getTotalNodes(long nativeApplication);
    public static native int getTotalWords(long nativeApplication);
    public static native int getTotalPoints(long nativeApplication);
    public static native float getUpdateTime(long nativeApplication);
    public static native int getLoopClosureId(long nativeApplication);
    
    public static native void postCameraPoseEvent(long nativeApplication, float x, float y, float z, float qx, float qy, float qz, float qw, double stamp);
    public static native void postOdometryEvent(long nativeApplication,
    		float x, float y, float z, float qx, float qy, float qz, float qw,
    		float rgb_fx, float rgb_fy, float rgb_cx, float rgb_cy,
    		float rgbFrameX, float rgbFrameY, float rgbFrameZ, float rgbFrameQX, float rgbFrameQY, float rgbFrameQZ, float rgbFrameQW,
    		double stamp, 
    		ByteBuffer yPlane, ByteBuffer uPlane, ByteBuffer vPlane, int yPlaneLen, int rgbWidth, int rgbHeight, int rgbFormat,
    		FloatBuffer points, int pointsLen,
    		float vx, float vy, float vz, float vqx, float vqy, float vqz, float vqw, //view matrix
            float p00, float p11, float p02, float p12, float p22, float p32, float p23, // projection matrix
            float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7); // tex coord
    public static native void postOdometryEventDepth(long nativeApplication,
    		float x, float y, float z, float qx, float qy, float qz, float qw,
    		float rgb_fx, float rgb_fy, float rgb_cx, float rgb_cy,
    		float depth_fx, float depth_fy, float depth_cx, float depth_cy,
    		float rgbFrameX, float rgbFrameY, float rgbFrameZ, float rgbFrameQX, float rgbFrameQY, float rgbFrameQZ, float rgbFrameQW,
    		float depthFrameX, float depthFrameY, float depthFrameZ, float depthFrameQX, float depthFrameQY, float depthFrameQZ, float depthFrameQW,
    		double rgbStamp,
    		double depthStamp,
    		ByteBuffer yPlane, ByteBuffer uPlane, ByteBuffer vPlane, int yPlaneLen, int rgbWidth, int rgbHeight, int rgbFormat,
    		ByteBuffer depth, int depthLen, int depthWidth, int depthHeight, int depthFormat,
    		FloatBuffer points, int pointsLen,
    		float vx, float vy, float vz, float vqx, float vqy, float vqz, float vqw, //view matrix
            float p00, float p11, float p02, float p12, float p22, float p32, float p23, // projection matrix
            float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7); // tex coord
    
}
