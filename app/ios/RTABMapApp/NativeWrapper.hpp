//
//  PCLWrapper.hpp
//  ThreeDScanner
//
//  Created by Steven Roach on 2/9/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

#ifndef NativeWrapper_hpp
#define NativeWrapper_hpp

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
    
const void *createNativeApplication();
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
                                                   float, float, float, float, float, float));
void destroyNativeApplication(const void *object);
void setScreenRotationNative(const void *object, int displayRotation);
int openDatabaseNative(const void *object, const char * databasePath, bool databaseInMemory, bool optimize, bool clearDatabase);
void saveNative(const void *object, const char * databasePath);
bool recoverNative(const void *object, const char * from, const char * to);
void cancelProcessingNative(const void * object);
int postProcessingNative(const void *object, int approach);
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
            bool blockRendering);
bool postExportationNative(const void *object, bool visualize);
bool writeExportedMeshNative(const void *object, const char * directory, const char * name);
void initGlContentNative(const void *object);
void setupGraphicNative(const void *object, int width, int height);
void onTouchEventNative(const void *object, int touch_count, int event, float x0, float y0, float x1,
                        float y1);
void setPausedMappingNative(const void *object, bool paused);
int renderNative(const void *object);
bool startCameraNative(const void *object);
void stopCameraNative(const void *object);
void setCameraNative(const void *object, int type);
void postCameraPoseEventNative(const void *object,
                         float x, float y, float z, float qx, float qy, float qz, float qw);
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
                       float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7);

void setOnlineBlendingNative(const void *object, bool enabled);
void setMapCloudShownNative(const void *object, bool shown);
void setOdomCloudShownNative(const void *object, bool shown);
void setMeshRenderingNative(const void *object, bool enabled, bool withTexture);
void setPointSizeNative(const void *object, float value);
void setFOVNative(const void *object, float angle);
void setOrthoCropFactorNative(const void *object, float value);
void setGridRotationNative(const void *object, float value);
void setLightingNative(const void *object, bool enabled);
void setBackfaceCullingNative(const void *object, bool enabled);
void setWireframeNative(const void *object, bool enabled);
void setLocalizationModeNative(const void *object, bool enabled);
void setTrajectoryModeNative(const void *object, bool enabled);
void setGraphOptimizationNative(const void *object, bool enabled);
void setNodesFilteringNative(const void *object, bool enabled);
void setGraphVisibleNative(const void *object, bool visible);
void setGridVisibleNative(const void *object, bool visible);
void setRawScanSavedNative(const void *object, bool enabled);
void setFullResolutionNative(const void *object, bool enabled);
void setSmoothingNative(const void *object, bool enabled);
void setAppendModeNative(const void *object, bool enabled);
void setMaxCloudDepthNative(const void *object, float value);
void setMinCloudDepthNative(const void *object, float value);
void setCloudDensityLevelNative(const void *object, int value);
void setMeshAngleToleranceNative(const void *object, float value);
void setMeshDecimationFactorNative(const void *object, float value);
void setMeshTriangleSizeNative(const void *object, int value);
void setClusterRatioNative(const void *object, float value);
void setMaxGainRadiusNative(const void *object, float value);
void setRenderingTextureDecimationNative(const void *object, int value);
void setBackgroundColorNative(const void *object, float gray);
void setDepthConfidenceNative(const void *object, int value);
int setMappingParameterNative(const void *object, const char * key, const char * value);

typedef struct ImageNative
{
    const void * objectPtr;
    void * data;
    int width;
    int height;
    int channels;
    int totalBytes;
} ImageNative;

ImageNative getPreviewImageNative(const char * databasePath);
void releasePreviewImageNative(ImageNative image);

void setGPSNative(const void *object, double stamp, double longitude, double latitude, double altitude, double accuracy, double bearing);
void addEnvSensorNative(const void *object, int type, float value);

#ifdef __cplusplus
}
#endif

#endif /* NativeWrapper_hpp */
