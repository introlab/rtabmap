
package com.introlab.rtabmap;
import android.view.KeyEvent;


// Wrapper for native library

public class RTABMapLib
{

    static
    {
      System.loadLibrary("NativeRTABMap");
    }

 // Initialize the Tango Service, this function starts the communication
    // between the application and Tango Service.
    // The activity object is used for checking if the API version is outdated.
    public static native int initialize(RTABMapActivity activity);
    
    public static native void openDatabase(String databasePath);
    
    public static native int onResume();

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
    public static native void setMapCloudShown(boolean shown);
    public static native void setOdomCloudShown(boolean shown);
    public static native void setMeshRendering(boolean enabled);
    public static native void setLocalizationMode(boolean enabled);
    public static native void setTrajectoryMode(boolean enabled);
    public static native void setGraphOptimization(boolean enabled);
    public static native void setNodesFiltering(boolean enabled);
    public static native void setGraphVisible(boolean visible);
    public static native void setAutoExposure(boolean enabled);
    public static native void setFullResolution(boolean enabled);
    public static native void setMaxCloudDepth(float value);
    public static native void setMeshAngleTolerance(float value);
    public static native void setMeshTriangleSize(int value);
    public static native int setMappingParameter(String key, String value);

    public static native void resetMapping();
    public static native void save();
    public static native boolean exportMesh(String filePath);
    public static native int postProcessing(int approach);
    
    public static native String getStatus();
    public static native int getTotalNodes();
    public static native int getTotalWords();
    public static native int getTotalPoints();
    public static native float getUpdateTime();
    public static native int getLoopClosureId();
    
}
