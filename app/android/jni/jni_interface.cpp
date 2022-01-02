/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define GLM_FORCE_RADIANS

#include <jni.h>
#include <RTABMapApp.h>
#include <scene.h>

#ifdef __cplusplus
extern "C" {
#endif

void GetJStringContent(JNIEnv *AEnv, jstring AStr, std::string &ARes) {
  if (!AStr) {
    ARes.clear();
    return;
  }

  const char *s = AEnv->GetStringUTFChars(AStr,NULL);
  ARes=s;
  AEnv->ReleaseStringUTFChars(AStr,s);
}

inline jlong jptr(RTABMapApp *native_computer_vision_application) {
  return reinterpret_cast<intptr_t>(native_computer_vision_application);
}

inline RTABMapApp *native(jlong ptr) {
  return reinterpret_cast<RTABMapApp *>(ptr);
}

JNIEXPORT jlong JNICALL
Java_com_introlab_rtabmap_RTABMapLib_createNativeApplication(
    JNIEnv* env, jclass, jobject activity)
{
	return jptr(new RTABMapApp(env, activity));
}

JNIEXPORT void Java_com_introlab_rtabmap_RTABMapLib_destroyNativeApplication(
	JNIEnv *, jclass, jlong native_application) {
	if(native_application)
	{
		delete native(native_application);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setScreenRotation(
    JNIEnv* env, jclass, jlong native_application, int displayRotation, int cameraRotation)
{
	if(native_application)
	{
		return native(native_application)->setScreenRotation(displayRotation, cameraRotation);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT int JNICALL
Java_com_introlab_rtabmap_RTABMapLib_openDatabase(
    JNIEnv* env, jclass, jlong native_application, jstring databasePath, bool databaseInMemory, bool optimize, bool clearDatabase)
{
	std::string databasePathC;
	GetJStringContent(env,databasePath,databasePathC);
	if(native_application)
	{
		return native(native_application)->openDatabase(databasePathC, databaseInMemory, optimize, clearDatabase);
	}
	else
	{
		UERROR("native_application is null!");
		return -1;
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_recover(
    JNIEnv* env, jclass, jlong native_application, jstring from, jstring to)
{
	if(native_application)
	{
		std::string toC;
		GetJStringContent(env,to,toC);
		std::string fromC;
		GetJStringContent(env,from,fromC);
		return native(native_application)->recover(fromC, toC);
	}
	else
	{
		UERROR("native_application is null!");
		return -1;
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_isBuiltWith(
		JNIEnv* env, jclass, jlong native_application, int cameraDriver) {
	if(native_application)
	{
		return native(native_application)->isBuiltWith(cameraDriver);
	}
	else
	{
		UERROR("native_application is null!");
		return false;
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_startCamera(
		JNIEnv* env, jclass, jlong native_application, jobject iBinder, jobject context, jobject activity, int driver) {
	if(native_application)
	{
		return native(native_application)->startCamera(env, iBinder, context, activity, driver);
	}
	else
	{
		UERROR("native_application is null!");
		return false;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_initGlContent(
    JNIEnv*, jclass, jlong native_application) {
	if(native_application)
	{
		native(native_application)->InitializeGLContent();
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setupGraphic(
    JNIEnv*, jclass, jlong native_application, jint width, jint height) {
	if(native_application)
	{
		native(native_application)->SetViewPort(width, height);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT int JNICALL
Java_com_introlab_rtabmap_RTABMapLib_render(
    JNIEnv*, jclass, jlong native_application) {
	if(native_application)
	{
		return native(native_application)->Render();
	}
	else
	{
		UERROR("native_application is null!");
		return -1;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_stopCamera(
    JNIEnv*, jclass, jlong native_application) {
	if(native_application)
	{
		native(native_application)->stopCamera();
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setCamera(
    JNIEnv*, jclass, jlong native_application, int camera_index) {
	if(native_application)
	{
		using namespace tango_gl;
		GestureCamera::CameraType cam_type =
		  static_cast<GestureCamera::CameraType>(camera_index);
		native(native_application)->SetCameraType(cam_type);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_onTouchEvent(
    JNIEnv*, jclass, jlong native_application, int touch_count, int event, float x0, float y0, float x1,
    float y1) {
	if(native_application)
	{
		using namespace tango_gl;
		GestureCamera::TouchEvent touch_event =
		  static_cast<GestureCamera::TouchEvent>(event);
		native(native_application)->OnTouchEvent(touch_count, touch_event, x0, y0, x1, y1);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setPausedMapping(
		JNIEnv*, jclass, jlong native_application, bool paused)
{
	if(native_application)
	{
		return native(native_application)->setPausedMapping(paused);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setOnlineBlending(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setOnlineBlending(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMapCloudShown(
		JNIEnv*, jclass, jlong native_application, bool shown)
{
	if(native_application)
	{
		return native(native_application)->setMapCloudShown(shown);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setOdomCloudShown(
		JNIEnv*, jclass, jlong native_application, bool shown)
{
	if(native_application)
	{
		return native(native_application)->setOdomCloudShown(shown);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshRendering(
		JNIEnv*, jclass, jlong native_application, bool enabled, bool withTexture)
{
	if(native_application)
	{
		return native(native_application)->setMeshRendering(enabled, withTexture);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setPointSize(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setPointSize(value);
	}
	else
	{
		UERROR("native_application is null!");
	}

}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setFOV(
		JNIEnv*, jclass, jlong native_application, float fov)
{
	if(native_application)
	{
		return native(native_application)->setFOV(fov);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setOrthoCropFactor(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setOrthoCropFactor(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGridRotation(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setGridRotation(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setLighting(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setLighting(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setBackfaceCulling(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setBackfaceCulling(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setWireframe(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setWireframe(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setLocalizationMode(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setLocalizationMode(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setTrajectoryMode(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setTrajectoryMode(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGraphOptimization(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setGraphOptimization(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setNodesFiltering(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setNodesFiltering(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGraphVisible(
		JNIEnv*, jclass, jlong native_application, bool visible)
{
	if(native_application)
	{
		return native(native_application)->setGraphVisible(visible);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGridVisible(
		JNIEnv*, jclass, jlong native_application, bool visible)
{
	if(native_application)
	{
		return native(native_application)->setGridVisible(visible);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setRawScanSaved(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setRawScanSaved(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setFullResolution(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setFullResolution(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setSmoothing(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setSmoothing(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setDepthFromMotion(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setDepthFromMotion(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setCameraColor(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setCameraColor(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setAppendMode(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setAppendMode(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setDataRecorderMode(
		JNIEnv*, jclass, jlong native_application, bool enabled)
{
	if(native_application)
	{
		return native(native_application)->setDataRecorderMode(enabled);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMaxCloudDepth(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setMaxCloudDepth(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMinCloudDepth(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setMinCloudDepth(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setCloudDensityLevel(
		JNIEnv*, jclass, jlong native_application, int value)
{
	if(native_application)
	{
		return native(native_application)->setCloudDensityLevel(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshAngleTolerance(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setMeshAngleTolerance(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshTriangleSize(
		JNIEnv*, jclass, jlong native_application, int value)
{
	if(native_application)
	{
		return native(native_application)->setMeshTriangleSize(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setClusterRatio(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setClusterRatio(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMaxGainRadius(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setMaxGainRadius(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setRenderingTextureDecimation(
		JNIEnv*, jclass, jlong native_application, int value)
{
	if(native_application)
	{
		return native(native_application)->setRenderingTextureDecimation(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setBackgroundColor(
		JNIEnv*, jclass, jlong native_application, float value)
{
	if(native_application)
	{
		return native(native_application)->setBackgroundColor(value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}
JNIEXPORT jint JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMappingParameter(
    JNIEnv* env, jclass, jlong native_application, jstring key, jstring value)
{
	if(native_application)
	{
		std::string keyC, valueC;
		GetJStringContent(env,key,keyC);
		GetJStringContent(env,value,valueC);
		return native(native_application)->setMappingParameter(keyC, valueC);
	}
	else
	{
		UERROR("native_application is null!");
		return -1;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGPS(
		JNIEnv*, jclass, jlong native_application,
		double stamp,
		double longitude,
		double latitude,
		double altitude,
		double accuracy,
		double bearing)
{
	if(native_application)
	{
		return native(native_application)->setGPS(rtabmap::GPS(stamp,
					longitude,
					latitude,
					altitude,
					accuracy,
					bearing));
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_addEnvSensor(
		JNIEnv*, jclass, jlong native_application,
		int type,
		float value)
{
	if(native_application)
	{
		return native(native_application)->addEnvSensor(type, value);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_save(
		JNIEnv* env, jclass, jlong native_application, jstring databasePath)
{
	if(native_application)
	{
		std::string databasePathC;
		GetJStringContent(env,databasePath,databasePathC);
		return native(native_application)->save(databasePathC);
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_cancelProcessing(
		JNIEnv* env, jclass, jlong native_application)
{
	if(native_application)
	{
		return native(native_application)->cancelProcessing();
	}
	else
	{
		UERROR("native_application is null!");
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_exportMesh(
		JNIEnv* env, jclass, jlong native_application,
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
	if(native_application)
	{
		return native(native_application)->exportMesh(
					cloudVoxelSize,
					regenerateCloud,
					meshing,
					textureSize,
					textureCount,
					normalK,
					optimized,
					optimizedVoxelSize,
					optimizedDepth,
					optimizedMaxPolygons,
					optimizedColorRadius,
					optimizedCleanWhitePolygons,
					optimizedMinClusterSize,
					optimizedMaxTextureDistance,
					optimizedMinTextureClusterSize,
					blockRendering);
	}
	else
	{
		UERROR("native_application is null!");
		return false;
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postExportation(
		JNIEnv* env, jclass, jlong native_application, bool visualize)
{
	if(native_application)
	{
		return native(native_application)->postExportation(visualize);
	}
	else
	{
		UERROR("native_application is null!");
		return false;
	}
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_writeExportedMesh(
		JNIEnv* env, jclass, jlong native_application, jstring directory, jstring name)
{
	if(native_application)
	{
		std::string directoryC;
		GetJStringContent(env,directory,directoryC);
		std::string nameC;
		GetJStringContent(env,name,nameC);
		return native(native_application)->writeExportedMesh(directoryC, nameC);
	}
	else
	{
		UERROR("native_application is null!");
		return false;
	}
}


JNIEXPORT int JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postProcessing(
		JNIEnv* env, jclass, jlong native_application, int approach)
{
	if(native_application)
	{
		return native(native_application)->postProcessing(approach);
	}
	else
	{
		UERROR("native_application is null!");
		return -1;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postCameraPoseEvent(
		JNIEnv* env, jclass, jlong native_application,
		float x, float y, float z, float qx, float qy, float qz, float qw, double stamp)
{
	if(native_application)
	{
		native(native_application)->postCameraPoseEvent(x,y,z,qx,qy,qz,qw, stamp);
	}
	else
	{
		UERROR("native_application is null!");
		return;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postOdometryEvent(
		JNIEnv* env, jclass, jlong native_application,
		float x, float y, float z, float qx, float qy, float qz, float qw,
		float rgb_fx, float rgb_fy, float rgb_cx, float rgb_cy,
		float rgbFrameX, float rgbFrameY, float rgbFrameZ, float rgbFrameQX, float rgbFrameQY, float rgbFrameQZ, float rgbFrameQW,
		double stamp,
		jobject yPlane, jobject uPlane, jobject vPlane, int yPlaneLen, int rgbWidth, int rgbHeight, int rgbFormat,
		jobject points, int pointsLen,
		float vx, float vy, float vz, float vqx, float vqy, float vqz, float vqw, //view matrix
		float p00, float p11, float p02, float p12, float p22, float p32, float p23, // projection matrix
		float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7) // tex coord
{
	if(native_application)
	{
		void *yPtr = env->GetDirectBufferAddress(yPlane);
		void *uPtr = env->GetDirectBufferAddress(uPlane);
		void *vPtr = env->GetDirectBufferAddress(vPlane);
		float *pointsPtr = (float *)env->GetDirectBufferAddress(points);
		native(native_application)->postOdometryEvent(
				rtabmap::Transform(x,y,z,qx,qy,qz,qw),
				rgb_fx,rgb_fy,rgb_cx,rgb_cy,
				0,0,0,0,
				rtabmap::Transform(rgbFrameX, rgbFrameY, rgbFrameZ, rgbFrameQX, rgbFrameQY, rgbFrameQZ, rgbFrameQW),
				rtabmap::Transform(),
				stamp,
				0,
				yPtr, uPtr, vPtr, yPlaneLen, rgbWidth, rgbHeight, rgbFormat,
				0,0,0,0,0, //depth
				0,0,0,0,0, //conf
				pointsPtr, pointsLen, 4,
				rtabmap::Transform(vx, vy, vz, vqx, vqy, vqz, vqw),
				p00, p11, p02, p12, p22, p32, p23,
				t0, t1, t2, t3, t4, t5, t6, t7);
	}
	else
	{
		UERROR("native_application is null!");
		return;
	}
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postOdometryEventDepth(
		JNIEnv* env, jclass, jlong native_application,
		float x, float y, float z, float qx, float qy, float qz, float qw,
		float rgb_fx, float rgb_fy, float rgb_cx, float rgb_cy,
		float depth_fx, float depth_fy, float depth_cx, float depth_cy,
		float rgbFrameX, float rgbFrameY, float rgbFrameZ, float rgbFrameQX, float rgbFrameQY, float rgbFrameQZ, float rgbFrameQW,
		float depthFrameX, float depthFrameY, float depthFrameZ, float depthFrameQX, float depthFrameQY, float depthFrameQZ, float depthFrameQW,
		double rgbStamp,
		double depthStamp,
		jobject yPlane, jobject uPlane, jobject vPlane, int yPlaneLen, int rgbWidth, int rgbHeight, int rgbFormat,
 		jobject depth, int depthLen, int depthWidth, int depthHeight, int depthFormat,
 		jobject points, int pointsLen,
		float vx, float vy, float vz, float vqx, float vqy, float vqz, float vqw, //view matrix
		float p00, float p11, float p02, float p12, float p22, float p32, float p23, // projection matrix
		float t0, float t1, float t2, float t3, float t4, float t5, float t6, float t7) // tex coord)
 {
		void *yPtr = env->GetDirectBufferAddress(yPlane);
		void *uPtr = env->GetDirectBufferAddress(uPlane);
		void *vPtr = env->GetDirectBufferAddress(vPlane);
 		void *depthPtr = env->GetDirectBufferAddress(depth);
 		float *pointsPtr = (float *)env->GetDirectBufferAddress(points);
 		native(native_application)->postOdometryEvent(
				rtabmap::Transform(x,y,z,qx,qy,qz,qw),
				rgb_fx,rgb_fy,rgb_cx,rgb_cy,
				depth_fx,depth_fy,depth_cx,depth_cy,
				rtabmap::Transform(rgbFrameX, rgbFrameY, rgbFrameZ, rgbFrameQX, rgbFrameQY, rgbFrameQZ, rgbFrameQW),
				rtabmap::Transform(depthFrameX, depthFrameY, depthFrameZ, depthFrameQX, depthFrameQY, depthFrameQZ, depthFrameQW),
				rgbStamp,
				depthStamp,
 				yPtr, uPtr, vPtr, yPlaneLen, rgbWidth, rgbHeight, rgbFormat,
 				depthPtr, depthLen, depthWidth, depthHeight, depthFormat,
				0,0,0,0,0, // conf
 				pointsPtr, pointsLen, 4,
				rtabmap::Transform(vx, vy, vz, vqx, vqy, vqz, vqw),
				p00, p11, p02, p12, p22, p32, p23,
				t0, t1, t2, t3, t4, t5, t6, t7);
 }


#ifdef __cplusplus
}
#endif
