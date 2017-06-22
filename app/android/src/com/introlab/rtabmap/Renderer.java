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

import java.util.Vector;
import java.util.concurrent.locks.ReentrantLock;

import android.app.Activity;
import android.app.ProgressDialog;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;
import android.widget.Toast;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

// Renderer renders graphic content. This includes the point cloud,
// ground grid, camera frustum, camera axis, and trajectory based on the Tango
// device's pose.
public class Renderer implements GLSurfaceView.Renderer {

	private final float[] mtrxProjection = new float[16];
	private final float[] mtrxView = new float[16];
	private final float[] mtrxProjectionAndView = new float[16];

	private TextManager mTextManager = null;
	private float mSurfaceHeight = 0.0f;
	private float mTextColor = 1.0f;
	private int mOffset = 0;
	
	private Vector<TextObject> mTexts;

	private static RTABMapActivity mActivity;
	public Renderer(RTABMapActivity c) {
		mActivity = c;
	}

	private ProgressDialog mProgressDialog = null;
	private Toast mToast = null;
	
	private boolean mTextChanged = false;
	private ReentrantLock mTextLock = new ReentrantLock();

	public void setProgressDialog(ProgressDialog progressDialog)
	{
		mProgressDialog = progressDialog;
	}

	public void setToast(Toast toast)
	{
		mToast = toast;
	}
	
	public void setOffset(int offset)
	{
		mOffset = offset;
	}

	// Render loop of the Gl context.
	public void onDrawFrame(GL10 useGLES20instead) {

		try
		{
			final int value = RTABMapLib.render();

			if(mTextManager!=null)
			{
				if(mTextChanged)
				{
					mTextChanged = false;
					Vector<TextObject> txtcollection = new Vector<TextObject>();
					
					mTextLock.lock();
				    try {
				    	if(mTexts.size() > 0)
				    	{
				    		txtcollection.addAll(mTexts);
				    	}
				    } finally {
				    	mTextLock.unlock();
				    }
				    
					// Prepare the text for rendering
					mTextManager.PrepareDraw(txtcollection);
				}
				
				float[] mvp = new float[16];
				Matrix.translateM(mvp, 0, mtrxProjectionAndView, 0, 0, mOffset, 0);
				mTextManager.Draw(mvp);
			}

			if(value != 0 && mProgressDialog != null && mProgressDialog.isShowing())
			{
				mActivity.runOnUiThread(new Runnable() {
					public void run() {
						if(!RTABMapActivity.DISABLE_LOG) Log.i("RTABMapActivity", "Renderer: dismiss dialog, value received=" + String.valueOf(value));
						mProgressDialog.dismiss();
						mActivity.resetNoTouchTimer();
					}
				});
			}
			if(value==-1)
			{
				mActivity.runOnUiThread(new Runnable() {
					public void run() {
						if(mToast!=null)
						{
							mToast.makeText(mActivity, String.format("Out of Memory!"), Toast.LENGTH_SHORT).show();
						}
					}
				});
			}
			else if(value==-2)
			{
				mActivity.runOnUiThread(new Runnable() {
					public void run() {
						if(mToast!=null)
						{
							mToast.makeText(mActivity, String.format("Rendering Error!"), Toast.LENGTH_SHORT).show();
						}
					}
				});
			}
		}
		catch(final Exception e)
		{
			mActivity.runOnUiThread(new Runnable() {
				public void run() {
					if(mToast!=null)
					{
						mToast.makeText(mActivity, String.format("Rendering error! %s", e.getMessage()), Toast.LENGTH_SHORT).show();
					}
				}

			});
		}
	}

	// Called when the surface size changes.
	public void onSurfaceChanged(GL10 useGLES20instead, int width, int height) {
		
		RTABMapLib.setupGraphic(width, height);
		
		mSurfaceHeight = (float)height;

		// Clear our matrices
		for(int i=0;i<16;i++)
		{
			mtrxProjection[i] = 0.0f;
			mtrxView[i] = 0.0f;
			mtrxProjectionAndView[i] = 0.0f;
		}

		// Setup our screen width and height for normal sprite translation.
		Matrix.orthoM(mtrxProjection, 0, 0f, width, 0.0f, height, 0, 50);

		// Set the camera position (View matrix)
		Matrix.setLookAtM(mtrxView, 0, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1.0f, 0.0f);

		// Calculate the projection and view transformation
		Matrix.multiplyMM(mtrxProjectionAndView, 0, mtrxProjection, 0, mtrxView, 0);
	}

	// Called when the surface is created or recreated.
	public void onSurfaceCreated(GL10 useGLES20instead, EGLConfig config) {

		RTABMapLib.initGlContent();
		
		// Create our text manager
		mTextManager = new TextManager(mActivity);
		mTextManager.setColor(mTextColor);
	}
	
	public void updateTexts(String[] texts)
	{
		if(mTextManager != null && mSurfaceHeight > 0.0f)
		{
			Vector<TextObject> textObjects = new Vector<TextObject>();
			float offset = mSurfaceHeight-mTextManager.getMaxTextHeight();
			if(texts != null)
			{
				for(int i=0;i<texts.length; ++i)
				{
					if(texts[i]!=null && texts[i].length()>0)
					{
						TextObject txt = new TextObject(texts[i], 0, offset);
						textObjects.add(txt);
					}
					offset-=mTextManager.getMaxTextHeight();
				}
			}
	
			mTextLock.lock();
			try {
				mTexts = textObjects;
			} finally {
				mTextLock.unlock();
			}
	
			mTextChanged = true;
		}
	}
	
	public void setTextColor(float color) 
	{
		mTextColor = color;
		if(mTextManager != null)
		{
			mTextManager.setColor(mTextColor);
		}
	}
}
