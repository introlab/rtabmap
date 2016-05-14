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

static RTABMapApp app;

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

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_onCreate(
    JNIEnv* env, jobject, jobject activity)
{
	return app.onCreate(env, activity);
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_openDatabase(
    JNIEnv* env, jobject, jstring databasePath)
{
	std::string databasePathC;
	GetJStringContent(env,databasePath,databasePathC);
	return app.openDatabase(databasePathC);
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_onTangoServiceConnected(
		JNIEnv* env, jobject, jobject iBinder) {
  return app.onTangoServiceConnected(env, iBinder);
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_initGlContent(
    JNIEnv*, jobject) {
  app.InitializeGLContent();
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setupGraphic(
    JNIEnv*, jobject, jint width, jint height) {
  app.SetViewPort(width, height);
}

JNIEXPORT int JNICALL
Java_com_introlab_rtabmap_RTABMapLib_render(
    JNIEnv*, jobject) {
  return app.Render();
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_onPause(
    JNIEnv*, jobject) {
  app.onPause();
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setCamera(
    JNIEnv*, jobject, int camera_index) {
  using namespace tango_gl;
  GestureCamera::CameraType cam_type =
      static_cast<GestureCamera::CameraType>(camera_index);
  app.SetCameraType(cam_type);
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_onTouchEvent(
    JNIEnv*, jobject, int touch_count, int event, float x0, float y0, float x1,
    float y1) {
  using namespace tango_gl;
  GestureCamera::TouchEvent touch_event =
      static_cast<GestureCamera::TouchEvent>(event);
  app.OnTouchEvent(touch_count, touch_event, x0, y0, x1, y1);
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setPausedMapping(
		JNIEnv*, jobject, bool paused)
{
	return app.setPausedMapping(paused);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMapCloudShown(
		JNIEnv*, jobject, bool shown)
{
	return app.setMapCloudShown(shown);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setOdomCloudShown(
		JNIEnv*, jobject, bool shown)
{
	return app.setOdomCloudShown(shown);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshRendering(
		JNIEnv*, jobject, bool enabled)
{
	return app.setMeshRendering(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setLocalizationMode(
		JNIEnv*, jobject, bool enabled)
{
	return app.setLocalizationMode(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setTrajectoryMode(
		JNIEnv*, jobject, bool enabled)
{
	return app.setTrajectoryMode(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGraphOptimization(
		JNIEnv*, jobject, bool enabled)
{
	return app.setGraphOptimization(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setNodesFiltering(
		JNIEnv*, jobject, bool enabled)
{
	return app.setNodesFiltering(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setGraphVisible(
		JNIEnv*, jobject, bool visible)
{
	return app.setGraphVisible(visible);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setAutoExposure(
		JNIEnv*, jobject, bool enabled)
{
	return app.setAutoExposure(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setFullResolution(
		JNIEnv*, jobject, bool enabled)
{
	return app.setFullResolution(enabled);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMaxCloudDepth(
		JNIEnv*, jobject, float value)
{
	return app.setMaxCloudDepth(value);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshAngleTolerance(
		JNIEnv*, jobject, float value)
{
	return app.setMeshAngleTolerance(value);
}
JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMeshTriangleSize(
		JNIEnv*, jobject, int value)
{
	return app.setMeshTriangleSize(value);
}
JNIEXPORT jint JNICALL
Java_com_introlab_rtabmap_RTABMapLib_setMappingParameter(
    JNIEnv* env, jobject, jstring key, jstring value)
{
	std::string keyC, valueC;
	GetJStringContent(env,key,keyC);
	GetJStringContent(env,value,valueC);
	return app.setMappingParameter(keyC, valueC);
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_resetMapping(
		JNIEnv*, jobject)
{
	return app.resetMapping();
}

JNIEXPORT void JNICALL
Java_com_introlab_rtabmap_RTABMapLib_save(
		JNIEnv* env, jobject)
{
	return app.save();
}

JNIEXPORT bool JNICALL
Java_com_introlab_rtabmap_RTABMapLib_exportMesh(
		JNIEnv* env, jobject, jstring filePath)
{
	std::string filePathC;
	GetJStringContent(env,filePath,filePathC);
	return app.exportMesh(filePathC);
}

JNIEXPORT int JNICALL
Java_com_introlab_rtabmap_RTABMapLib_postProcessing(
		JNIEnv* env, jobject, int approach)
{
	return app.postProcessing(approach);
}



#ifdef __cplusplus
}
#endif
