//
//  PCLWrapper.cpp
//  ThreeDScanner
//
//  Created by Steven Roach on 2/9/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

#include "NativeWrapper.hpp"
#include "RTABMapApp.h"
#include <rtabmap/core/DBDriverSqlite3.h>

inline RTABMapApp *native(const void *object) {
  return (RTABMapApp *)object;
}

const void * createNativeApplication()
{
    RTABMapApp *app = new RTABMapApp();
    return (void *)app;
}

void setupCallbacksNative(const void *object, void * classPtr,
                          void(*progressCallback)(void*, int, int),
                          void(*initCallback)(void *, int, const char*),
                          void(*statsUpdatedCallback)(void *,
                                                   int, int, int, int,
                                                   float,
                                                   int, int, int, int, int ,int,
                                                   float,
                                                   int,
                                                   float,
                                                   int,
                                                   float, float, float, float,
                                                   int, int,
                                                   float, float, float, float, float, float))
{
    if(object)
    {
        native(object)->setupSwiftCallbacks(classPtr, progressCallback, initCallback, statsUpdatedCallback);
    }
    else
    {
        UERROR("object is null!");
    }
}

void destroyNativeApplication(const void *object)
{
    if(object)
    {
        delete native(object);
    }
    else
    {
        UERROR("object is null!");
    }
}

void setScreenRotationNative(const void *object, int displayRotation)
{
    if(object)
    {
        return native(object)->setScreenRotation(displayRotation, 0);
    }
    else
    {
        UERROR("object is null!");
    }
}

int openDatabaseNative(const void *object, const char * databasePath, bool databaseInMemory, bool optimize, bool clearDatabase)
{
    if(object)
    {
        return native(object)->openDatabase(databasePath, databaseInMemory, optimize, clearDatabase);
    }
    else
    {
        UERROR("object is null!");
        return -1;
    }
}

void saveNative(const void *object, const char * databasePath)
{
    if(object)
    {
        return native(object)->save(databasePath);
    }
    else
    {
        UERROR("object is null!");
    }
}

bool recoverNative(const void *object, const char * from, const char * to)
{
    if(object)
    {
        return native(object)->recover(from, to);
    }
    else
    {
        UERROR("object is null!");
    }
    return false;
}

void cancelProcessingNative(const void *object)
{
    if(object)
    {
        native(object)->cancelProcessing();
    }
    else
    {
        UERROR("object is null!");
    }
}

int postProcessingNative(const void *object, int approach)
{
    if(object)
    {
        return native(object)->postProcessing(approach);
    }
    else
    {
        UERROR("object is null!");
    }
    return -1;
}

bool exportMeshNative(
            const void *object,
            float cloudVoxelSize,
            bool regenerateCloud,
            bool meshing,
            int textureSize,
            int textureCount,
            int normalK,
            bool optimized,
            float optimizedVoxelSize,
            int optimizedDepth,
            int optimizedMaxPolygons,
            float optimizedColorRadius,
            bool optimizedCleanWhitePolygons,
            int optimizedMinClusterSize,
            float optimizedMaxTextureDistance,
            int optimizedMinTextureClusterSize,
            bool blockRendering)
{
    if(object)
    {
        return native(object)->exportMesh(cloudVoxelSize, regenerateCloud, meshing, textureSize, textureCount, normalK, optimized, optimizedVoxelSize, optimizedDepth, optimizedMaxPolygons, optimizedColorRadius, optimizedCleanWhitePolygons, optimizedMinClusterSize, optimizedMaxTextureDistance, optimizedMinTextureClusterSize, blockRendering);
    }
    else
    {
        UERROR("object is null!");
    }
    return false;
}

bool postExportationNative(const void *object, bool visualize)
{
    if(object)
    {
        return native(object)->postExportation(visualize);
    }
    else
    {
        UERROR("object is null!");
    }
    return false;
}

bool writeExportedMeshNative(const void *object, const char * directory, const char * name)
{
    if(object)
    {
        return native(object)->writeExportedMesh(directory, name);
    }
    else
    {
        UERROR("object is null!");
    }
    return false;
}

void initGlContentNative(const void *object) {
    if(object)
    {
        native(object)->InitializeGLContent();
    }
    else
    {
        UERROR("object is null!");
    }
}

void setupGraphicNative(const void *object, int width, int height) {
    if(object)
    {
        native(object)->SetViewPort(width, height);
    }
    else
    {
        UERROR("object is null!");
    }
}

void onTouchEventNative(const void *object, int touch_count, int event, float x0, float y0, float x1,
    float y1) {
    if(object)
    {
        using namespace tango_gl;
        GestureCamera::TouchEvent touch_event =
          static_cast<GestureCamera::TouchEvent>(event);
        native(object)->OnTouchEvent(touch_count, touch_event, x0, y0, x1, y1);
    }
    else
    {
        UERROR("object is null!");
    }
}

void setPausedMappingNative(const void *object, bool paused)
{
    if(object)
    {
        return native(object)->setPausedMapping(paused);
    }
    else
    {
        UERROR("object is null!");
    }
}

int renderNative(const void *object) {
    if(object)
    {
        return native(object)->Render();
    }
    else
    {
        UERROR("object is null!");
        return -1;
    }
}

bool startCameraNative(const void *object) {
    if(object)
    {
        return native(object)->startCamera();
    }
    else
    {
        UERROR("object is null!");
        return false;
    }
}

void stopCameraNative(const void *object) {
    if(object)
    {
        native(object)->stopCamera();
    }
    else
    {
        UERROR("object is null!");
    }
}

void setCameraNative(const void *object, int type) {
    if(object)
    {
        native(object)->SetCameraType(tango_gl::GestureCamera::CameraType(type));
    }
    else
    {
        UERROR("object is null!");
    }
}

void postCameraPoseEventNative(const void *object,
        float x, float y, float z, float qx, float qy, float qz, float qw)
{
    if(object)
    {
        native(object)->postCameraPoseEvent(x,y,z,qx,qy,qz,qw,0.0);
    }
    else
    {
        UERROR("object is null!");
        return;
    }
}

void postOdometryEventNative(const void *object,
        float x, float y, float z, float qx, float qy, float qz, float qw,
        float fx, float fy, float cx, float cy,
        double stamp,
        const void * yPlane,  const void * uPlane,  const void * vPlane, int yPlaneLen, int rgbWidth, int rgbHeight, int rgbFormat,
        const void * depth, int depthLen, int depthWidth, int depthHeight, int depthFormat,
        const void * conf, int confLen, int confWidth, int confHeight, int confFormat,
        const void * points, int pointsLen, int pointsChannels,
        float vx, float vy, float vz, float vqx, float vqy, float vqz, float vqw,
        float p00, float p11, float p02, float p12, float p22, float p32, float p23,
        float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7)
{
    if(object)
    {
        native(object)->postOdometryEvent(
                rtabmap::Transform(x,y,z,qx,qy,qz,qw),
                fx,fy,cx,cy, 0,0,0,0,
                rtabmap::Transform(), rtabmap::Transform(),
                stamp, 0,
                yPlane, uPlane, vPlane, yPlaneLen, rgbWidth, rgbHeight, rgbFormat,
                depth, depthLen, depthWidth, depthHeight, depthFormat,
                conf, confLen, confWidth, confHeight, confFormat,
                (const float *)points, pointsLen, pointsChannels,
                rtabmap::Transform(vx, vy, vz, vqx, vqy, vqz, vqw),
                p00, p11, p02, p12, p22, p32, p23,
                t0, t1, t2, t3, t4, t5, t6, t7);
    }
    else
    {
        UERROR("object is null!");
        return;
    }
}

ImageNative getPreviewImageNative(const char * databasePath)
{
    ImageNative imageNative;
    imageNative.data = 0;
    imageNative.objectPtr = 0;
    rtabmap::DBDriverSqlite3 driver;
    if(driver.openConnection(databasePath))
    {
        cv::Mat image = driver.loadPreviewImage();
        if(image.empty())
        {
            return imageNative;
        }
        cv::Mat * imagePtr = new cv::Mat();
        // We should add alpha channel
        cv::cvtColor(image, *imagePtr, cv::COLOR_BGR2BGRA);
        std::vector<cv::Mat> channels;
        cv::split(*imagePtr, channels);
        channels.back() = cv::Scalar(255);
        cv::merge(channels, *imagePtr);
        imageNative.objectPtr = imagePtr;
        imageNative.data = imagePtr->data;
        imageNative.width = imagePtr->cols;
        imageNative.height = imagePtr->rows;
        imageNative.channels = imagePtr->channels();
    }
    return imageNative;
}

void releasePreviewImageNative(ImageNative image)
{
    if(image.objectPtr)
    {
        delete (cv::Mat*)image.objectPtr;
    }
}


// Parameters
void setOnlineBlendingNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setOnlineBlending(enabled);
    else
        UERROR("object is null!");
}
void setMapCloudShownNative(const void *object, bool shown)
{
    if(object)
        native(object)->setMapCloudShown(shown);
    else
        UERROR("object is null!");
}
void setOdomCloudShownNative(const void *object, bool shown)
{
    if(object)
        native(object)->setOdomCloudShown(shown);
    else
        UERROR("object is null!");
}
void setMeshRenderingNative(const void *object, bool enabled, bool withTexture)
{
    if(object)
        native(object)->setMeshRendering(enabled, withTexture);
    else
        UERROR("object is null!");
}
void setPointSizeNative(const void *object, float value)
{
    if(object)
        native(object)->setPointSize(value);
    else
        UERROR("object is null!");
}
void setFOVNative(const void *object, float angle)
{
    if(object)
        native(object)->setFOV(angle);
    else
        UERROR("object is null!");
}
void setOrthoCropFactorNative(const void *object, float value)
{
    if(object)
        native(object)->setOrthoCropFactor(value);
    else
        UERROR("object is null!");
}
void setGridRotationNative(const void *object, float value)
{
    if(object)
        native(object)->setGridRotation(value);
    else
        UERROR("object is null!");
}
void setLightingNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setLighting(enabled);
    else
        UERROR("object is null!");
}
void setBackfaceCullingNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setBackfaceCulling(enabled);
    else
        UERROR("object is null!");
}
void setWireframeNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setWireframe(enabled);
    else
        UERROR("object is null!");
}
void setLocalizationModeNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setLocalizationMode(enabled);
    else
        UERROR("object is null!");
}
void setTrajectoryModeNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setTrajectoryMode(enabled);
    else
        UERROR("object is null!");
}
void setGraphOptimizationNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setGraphOptimization(enabled);
    else
        UERROR("object is null!");
}
void setNodesFilteringNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setNodesFiltering(enabled);
    else
        UERROR("object is null!");
}
void setGraphVisibleNative(const void *object, bool visible)
{
    if(object)
        native(object)->setGraphVisible(visible);
    else
        UERROR("object is null!");
}
void setGridVisibleNative(const void *object, bool visible)
{
    if(object)
        native(object)->setGridVisible(visible);
    else
        UERROR("object is null!");
}
void setRawScanSavedNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setRawScanSaved(enabled);
    else
        UERROR("object is null!");
}
void setFullResolutionNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setFullResolution(enabled);
    else
        UERROR("object is null!");
}
void setSmoothingNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setSmoothing(enabled);
    else
        UERROR("object is null!");
}
void setAppendModeNative(const void *object, bool enabled)
{
    if(object)
        native(object)->setAppendMode(enabled);
    else
        UERROR("object is null!");
}
void setMaxCloudDepthNative(const void *object, float value)
{
    if(object)
        native(object)->setMaxCloudDepth(value);
    else
        UERROR("object is null!");
}
void setMinCloudDepthNative(const void *object, float value)
{
    if(object)
        native(object)->setMinCloudDepth(value);
    else
        UERROR("object is null!");
}
void setCloudDensityLevelNative(const void *object, int value)
{
    if(object)
        native(object)->setCloudDensityLevel(value);
    else
        UERROR("object is null!");
}
void setMeshAngleToleranceNative(const void *object, float value)
{
    if(object)
        native(object)->setMeshAngleTolerance(value);
    else
        UERROR("object is null!");
}
void setMeshDecimationFactorNative(const void *object, float value)
{
    if(object)
        native(object)->setMeshDecimationFactor(value);
    else
        UERROR("object is null!");
}
void setMeshTriangleSizeNative(const void *object, int value)
{
    if(object)
        native(object)->setMeshTriangleSize(value);
    else
        UERROR("object is null!");
}
void setClusterRatioNative(const void *object, float value)
{
    if(object)
        native(object)->setClusterRatio(value);
    else
        UERROR("object is null!");
}
void setMaxGainRadiusNative(const void *object, float value)
{
    if(object)
        native(object)->setMaxGainRadius(value);
    else
        UERROR("object is null!");
}
void setRenderingTextureDecimationNative(const void *object, int value)
{
    if(object)
        native(object)->setRenderingTextureDecimation(value);
    else
        UERROR("object is null!");
}
void setBackgroundColorNative(const void *object, float gray)
{
    if(object)
        native(object)->setBackgroundColor(gray);
    else
        UERROR("object is null!");
}
void setDepthConfidenceNative(const void *object, int value)
{
    if(object)
        native(object)->setDepthConfidence(value);
    else
        UERROR("object is null!");
}
int setMappingParameterNative(const void *object, const char * key, const char * value)
{
    if(object)
        return native(object)->setMappingParameter(key, value);
    else
        UERROR("object is null!");
    return -1;
}

void setGPSNative(const void *object, double stamp, double longitude, double latitude, double altitude, double accuracy, double bearing)
{
    rtabmap::GPS gps(stamp, longitude, latitude, altitude, accuracy, bearing);
    if(object)
        return native(object)->setGPS(gps);
    else
        UERROR("object is null!");
}

void addEnvSensorNative(const void *object, int type, float value)
{
    if(object)
        return native(object)->addEnvSensor(type, value);
    else
        UERROR("object is null!");
}
