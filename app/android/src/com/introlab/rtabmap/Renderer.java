/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.introlab.rtabmap;

import android.app.Activity;
import android.app.ProgressDialog;
import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.Log;
import android.widget.Toast;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// Renderer renders graphic content. This includes the point cloud,
// ground grid, camera frustum, camera axis, and trajectory based on the Tango
// device's pose.
public class Renderer implements GLSurfaceView.Renderer {
	
	private static Activity mActivity;
	  public Renderer(Activity c) {
		  mActivity = c;
	  }

	private ProgressDialog mProgressDialog;
	
	public void setProgressDialog(ProgressDialog progressDialog)
	{
		mProgressDialog = progressDialog;
	}
	
	// Render loop of the Gl context.
	public void onDrawFrame(GL10 gl) {
		try
		{
			final int value = RTABMapLib.render();
			mActivity.runOnUiThread(new Runnable() {
				public void run() {
					if(value != 0 && mProgressDialog != null && mProgressDialog.isShowing())
					{
						Log.i("RTABMapActivity", "Renderer: dismiss dialog, value received=" + String.valueOf(value));
						mProgressDialog.dismiss();
					}
					if(value==-1)
					{
						Toast.makeText(mActivity, String.format("Out of Memory!"), Toast.LENGTH_LONG).show();
					}
				} 
			});
		}
		catch(final Exception e)
		{
			mActivity.runOnUiThread(new Runnable() {
				public void run() {
					Toast.makeText(mActivity, String.format("Rendering error! %s", e.getMessage()), Toast.LENGTH_LONG).show();
				}

			});
		}
	}

  // Called when the surface size changes.
  public void onSurfaceChanged(GL10 gl, int width, int height) {
	  RTABMapLib.setupGraphic(width, height);
  }

  // Called when the surface is created or recreated.
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {
	  RTABMapLib.initGlContent();
  }
}
