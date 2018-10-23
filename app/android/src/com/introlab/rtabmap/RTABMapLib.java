
package com.introlab.rtabmap;
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
            Log.e(RTABMapActivity.class.getSimpleName(), "ERROR! Unable to load libtango_client_api.so!");
        }
        System.loadLibrary("NativeRTABMap");
    }

 // Initialize the Tango Service, this function starts the communication
    // between the application and Tango Service.
    // The activity object is used for checking if the API version is outdated.
    public static native void onCreate(RTABMapActivity activity);
    
    public static native void setScreenRotation(int displayRotation, int cameraRotation);
    
    public static native int openDatabase(String databasePath, boolean databaseInMemory, boolean optimize);
    public static native int openDatabase2(String databaseSource, String databasePath, boolean databaseInMemory, boolean optimize);
    
    /*
     * Called when the Tango service is connected.
     *
     * @param binder The native binder object.
     */
    public static native boolean onTangoServiceConnected(IBinder binder);

    // Release all non OpenGl resources that are allocated from the program.
    public static native void onPause();

    // Allocate OpenGL resources for rendering.
    public static native void initGlContent();

    // Setup the view port width and height.
    public static native void setupGraphic(int width, int height);

    // Main render loop.
    public static native int render();

    // Set the render camera's viewing angle:
    //   first person, third person, or top down.
    public static native void setCamera(int cameraIndex);
    
    // Pass touch events to the native layer.
    public static native void onTouchEvent(int touchCount, int event0,
                                           float x0, float y0, float x1, float y1);
    

    public static native void setPausedMapping(boolean paused);
    public static native void setOnlineBlending(boolean enabled);
    public static native void setMapCloudShown(boolean shown);
    public static native void setOdomCloudShown(boolean shown);
    public static native void setMeshRendering(boolean enabled, boolean withTexture);
    public static native void setLocalizationMode(boolean enabled);
    public static native void setTrajectoryMode(boolean enabled);
    public static native void setGraphOptimization(boolean enabled);
    public static native void setNodesFiltering(boolean enabled);
    public static native void setGraphVisible(boolean visible);
    public static native void setGridVisible(boolean visible);
    public static native void setRawScanSaved(boolean enabled);
    public static native void setFullResolution(boolean enabled);
    public static native void setSmoothing(boolean enabled);
    public static native void setCameraColor(boolean enabled);
    public static native void setAppendMode(boolean enabled);
    public static native void setDataRecorderMode(boolean enabled);
    public static native void setMaxCloudDepth(float value);
    public static native void setMinCloudDepth(float value);
    public static native void setPointSize(float value);
    public static native void setFOV(float value);
    public static native void setOrthoCropFactor(float value);
    public static native void setGridRotation(float value);
    public static native void setLighting(boolean enabled);
    public static native void setBackfaceCulling(boolean enabled);
    public static native void setWireframe(boolean enabled);
    public static native void setCloudDensityLevel(int value);
    public static native void setMeshAngleTolerance(float value);
    public static native void setMeshTriangleSize(int value);
    public static native void setClusterRatio(float value);
    public static native void setMaxGainRadius(float value);
    public static native void setRenderingTextureDecimation(int value);
    public static native void setBackgroundColor(float gray);
    public static native int setMappingParameter(String key, String value);
    public static native void setGPS(
			double stamp,
			double longitude, 
			double latitude, 
			double altitude,  
			double accuracy,
			double bearing);
    public static native void addEnvSensor(int type, float value);

    public static native void resetMapping();
    public static native void save(String outputDatabasePath);
    public static native void cancelProcessing();
    public static native boolean exportMesh(
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
    public static native boolean writeExportedMesh(String directory, String name);
    public static native boolean postExportation(boolean visualize);
    public static native int postProcessing(int approach);
    
    public static native String getStatus();
    public static native int getTotalNodes();
    public static native int getTotalWords();
    public static native int getTotalPoints();
    public static native float getUpdateTime();
    public static native int getLoopClosureId();
    
}
