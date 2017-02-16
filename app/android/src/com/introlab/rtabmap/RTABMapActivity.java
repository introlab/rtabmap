package com.introlab.rtabmap;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import android.app.Activity;
import android.app.ActivityManager;
import android.app.ActivityManager.MemoryInfo;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.ProgressDialog;
import android.content.ComponentName;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.hardware.Camera;
import android.graphics.Point;
import android.hardware.display.DisplayManager;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Debug;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.InputType;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.google.atap.tangoservice.Tango;

// The main activity of the application. This activity shows debug information
// and a glSurfaceView that renders graphic content.
public class RTABMapActivity extends Activity implements OnClickListener {

	// Tag for debug logging.
	public static final String TAG = RTABMapActivity.class.getSimpleName();

	// The minimum Tango Core version required from this application.
	private static final int  MIN_TANGO_CORE_VERSION = 9377;

	// The package name of Tang Core, used for checking minimum Tango Core version.
	private static final String TANGO_PACKAGE_NAME = "com.google.tango";

	public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
	public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";
	
	public static final String RTABMAP_TMP_DB = "rtabmap.tmp.db";
	public static final String RTABMAP_TMP_DIR = "tmp/";
	public static final String RTABMAP_TMP_FILENAME = "map";

	public static final String RTABMAP_AUTH_TOKEN_KEY = "com.introlab.rtabmap.AUTH_TOKEN";
	public static final String RTABMAP_FILENAME_KEY = "com.introlab.rtabmap.FILENAME";
	public static final String RTABMAP_OPENED_DB_PATH_KEY = "com.introlab.rtabmap.OPENED_DB_PATH";
	public static final String RTABMAP_EXPORTED_OBJ_KEY = "com.introlab.rtabmap.EXPORTED_OBJ";
	public static final String RTABMAP_WORKING_DIR_KEY = "com.introlab.rtabmap.WORKING_DIR";
	public static final int SKETCHFAB_ACTIVITY_CODE = 999;
	private String mAuthToken;

	// UI states
	private static enum State {
		STATE_IDLE,
		STATE_PROCESSING,
		STATE_VISUALIZING
	}
	State mState = State.STATE_IDLE;

	// GLSurfaceView and renderer, all of the graphic content is rendered
	// through OpenGL ES 2.0 in native code.
	private Renderer mRenderer;
	private GLSurfaceView mGLView;

	ProgressDialog mProgressDialog;

	// Screen size for normalizing the touch input for orbiting the render camera.
	private Point mScreenSize = new Point();
	private long mOnPauseStamp = 0;
	private boolean mOnPause = false;
	private Date mDateOnPause = new Date();

	private MenuItem mItemSave;
	private MenuItem mItemOpen;
	private MenuItem mItemPostProcessing;
	private MenuItem mItemExport;
	private MenuItem mItemSettings;
	private MenuItem mItemModes;
	private MenuItem mItemReset;
	private MenuItem mItemLocalizationMode;
	private MenuItem mItemTrajectoryMode;
	private MenuItem mItemRenderingPointCloud;
	private MenuItem mItemRenderingMesh;
	private MenuItem mItemRenderingTextureMesh;
	private MenuItem mItemDataRecorderMode;

	private ToggleButton mButtonFirst;
	private ToggleButton mButtonThird;
	private ToggleButton mButtonTop;
	private ToggleButton mButtonPause;
	private ToggleButton mButtonLighting;
	private ToggleButton mButtonBackfaceShown;
	private Button mButtonCloseVisualization;
	private Button mButtonSaveOnDevice;
	private Button mButtonShareOnSketchfab;

	private String mOpenedDatabasePath = "";
	private String mWorkingDirectory = "";

	private String mUpdateRate;
	private String mTimeThr;
	private String mMaxFeatures;
	private String mLoopThr;
	private String mMinInliers;
	private String mMaxOptimizationError;

	private LinearLayout mLayoutDebug;

	private int mTotalLoopClosures = 0;
	private boolean mMapIsEmpty = false;
	private boolean mExportedOBJ = false;

	private Toast mToast = null;
	
	private Thread mMemStatusUpdateThread = null;

	//Tango Service connection.
	ServiceConnection mTangoServiceConnection = new ServiceConnection() {
		public void onServiceConnected(ComponentName name, final IBinder service) {
			Thread bindThread = new Thread(new Runnable() {
				public void run() {
					if(!RTABMapLib.onTangoServiceConnected(service))
					{
						runOnUiThread(new Runnable() {
							public void run() {
								mToast.makeText(getApplicationContext(), 
										String.format("Failed to intialize Tango!"), mToast.LENGTH_LONG).show();
							} 
						});
					}
				}
			});
			bindThread.start();
		}

		public void onServiceDisconnected(ComponentName name) {
			// Handle this if you need to gracefully shutdown/retry
			// in the event that Tango itself crashes/gets upgraded while running.
			mToast.makeText(getApplicationContext(), 
					String.format("Tango disconnected!"), mToast.LENGTH_LONG).show();
		}
	};

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle(R.string.menu_name);

		// Query screen size, the screen size is used for computing the normalized
		// touch point.
		Display display = getWindowManager().getDefaultDisplay();
		display.getSize(mScreenSize);

		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		// Setting content view of this activity.
		setContentView(R.layout.activity_rtabmap);

		// Make sure to initialize all default values
		SettingsActivity settings;

		// Buttons for selecting camera view and Set up button click listeners.
		mButtonFirst = (ToggleButton)findViewById(R.id.first_person_button);
		mButtonThird = (ToggleButton)findViewById(R.id.third_person_button);
		mButtonTop = (ToggleButton)findViewById(R.id.top_down_button);
		mButtonPause = (ToggleButton)findViewById(R.id.pause_button);
		mButtonLighting = (ToggleButton)findViewById(R.id.light_button);
		mButtonBackfaceShown = (ToggleButton)findViewById(R.id.backface_button);
		mButtonCloseVisualization = (Button)findViewById(R.id.close_visualization_button);
		mButtonSaveOnDevice = (Button)findViewById(R.id.button_saveOnDevice);
		mButtonShareOnSketchfab = (Button)findViewById(R.id.button_shareToSketchfab);
		mButtonFirst.setOnClickListener(this);
		mButtonThird.setOnClickListener(this);
		mButtonTop.setOnClickListener(this);
		mButtonPause.setOnClickListener(this);
		mButtonLighting.setOnClickListener(this);
		mButtonBackfaceShown.setOnClickListener(this);
		mButtonCloseVisualization.setOnClickListener(this);
		mButtonSaveOnDevice.setOnClickListener(this);
		mButtonShareOnSketchfab.setOnClickListener(this);
		mButtonFirst.setChecked(true);
		mButtonLighting.setChecked(true);
		mButtonLighting.setVisibility(View.INVISIBLE);
		mButtonCloseVisualization.setVisibility(View.INVISIBLE);
		mButtonSaveOnDevice.setVisibility(View.INVISIBLE);
		mButtonShareOnSketchfab.setVisibility(View.INVISIBLE);
		if(mItemRenderingMesh != null && mItemRenderingTextureMesh != null)
		{
			mButtonBackfaceShown.setVisibility(mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked()?View.VISIBLE:View.INVISIBLE);
		}

		mToast = Toast.makeText(getApplicationContext(), "", Toast.LENGTH_SHORT);

		// OpenGL view where all of the graphics are drawn.
		mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

		// Configure OpenGL renderer
		mGLView.setEGLContextClientVersion(2);

		// Configure the OpenGL renderer.
		mRenderer = new Renderer(this);
		mGLView.setRenderer(mRenderer);

		mLayoutDebug = (LinearLayout) findViewById(R.id.debug_layout);
		mLayoutDebug.setVisibility(LinearLayout.GONE);

		mProgressDialog = new ProgressDialog(this);
		mProgressDialog.setCanceledOnTouchOutside(false);
		mRenderer.setProgressDialog(mProgressDialog);

		// Check if the Tango Core is out dated.
		if (!CheckTangoCoreVersion(MIN_TANGO_CORE_VERSION)) {
			mToast.makeText(this, "Tango Core out dated, please update in Play Store", mToast.LENGTH_LONG).show();
			finish();
			return;
		}   

		mOpenedDatabasePath = "";
		mWorkingDirectory = "";
		mTotalLoopClosures = 0;

		if(Environment.getExternalStorageState().compareTo(Environment.MEDIA_MOUNTED)==0)
		{
			File extStore = Environment.getExternalStorageDirectory();
			mWorkingDirectory = extStore.getAbsolutePath() + "/" + getString(R.string.app_name) + "/";
			extStore = new File(mWorkingDirectory);
			extStore.mkdirs();
		}
		else
		{
			// show warning that data cannot be saved!
			mToast.makeText(getApplicationContext(), 
					String.format("Failed to get external storage path (SD-CARD, state=%s). Saving disabled.", 
							Environment.getExternalStorageState()), mToast.LENGTH_LONG).show();
		}

		RTABMapLib.onCreate(this);
		String tmpDatabase = mWorkingDirectory+RTABMAP_TMP_DB;
		(new File(tmpDatabase)).delete();
		SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
		boolean databaseInMemory = sharedPref.getBoolean(getString(R.string.pref_key_db_in_memory), Boolean.parseBoolean(getString(R.string.pref_default_db_in_memory)));
		RTABMapLib.openDatabase(tmpDatabase, databaseInMemory, false);

		DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {

                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setAndroidOrientation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {}
            }, null);
        }
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		// Check which request we're responding to
		if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
			// Make sure the request was successful
			if (resultCode == RESULT_CANCELED) {
				mToast.makeText(this, "Motion Tracking Permissions Required!",
						mToast.LENGTH_SHORT).show();
				finish();
			}
		}
		else if (requestCode == SKETCHFAB_ACTIVITY_CODE) {
			// Make sure the request was successful
			if (resultCode == RESULT_OK) {
				mAuthToken = data.getStringExtra(RTABMAP_AUTH_TOKEN_KEY);
			}
		}
	}
	
	@Override
	protected void onPause() {
		super.onPause();
		
		Log.i(TAG, "onPause()");
		mOnPause = true;
		
		// This deletes OpenGL context!
		mGLView.onPause();

		RTABMapLib.onPause();

		unbindService(mTangoServiceConnection);

		if(!mButtonPause.isChecked())
		{
			mButtonPause.setChecked(true);
			pauseMapping();
		}
		
		mOnPauseStamp = System.currentTimeMillis()/1000;
	}

	@Override
	protected void onResume() {
		super.onResume();
		
		mProgressDialog.setTitle("");
		if(mOnPause)
		{
			if(System.currentTimeMillis()/1000 - mOnPauseStamp < 1)
			{
				mProgressDialog.setMessage(String.format("RTAB-Map has been interrupted by another application, Tango should be re-initialized! Set your phone/tablet in Airplane mode if this happens often."));
			}
			else
			{
				mProgressDialog.setMessage(String.format("Hold Tight! Initializing Tango Service..."));
			}
			mToast.makeText(this, "Mapping is paused!", mToast.LENGTH_LONG).show();
		}
		else
		{
			mProgressDialog.setMessage(String.format("Hold Tight! Initializing Tango Service...\nTip: If the camera is still drifting just after the mapping has started, do \"Reset\"."));
		}
		mProgressDialog.show();
		mOnPause = false;
		
		setAndroidOrientation();

		// update preferences
		try
		{
			Log.d(TAG, "update preferences...");
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			mUpdateRate = sharedPref.getString(getString(R.string.pref_key_update_rate), getString(R.string.pref_default_update_rate));
			mTimeThr = sharedPref.getString(getString(R.string.pref_key_time_thr), getString(R.string.pref_default_time_thr));
			String memThr = sharedPref.getString(getString(R.string.pref_key_mem_thr), getString(R.string.pref_default_mem_thr));
			mLoopThr = sharedPref.getString(getString(R.string.pref_key_loop_thr), getString(R.string.pref_default_loop_thr));
			String simThr = sharedPref.getString(getString(R.string.pref_key_sim_thr), getString(R.string.pref_default_sim_thr));
			mMinInliers = sharedPref.getString(getString(R.string.pref_key_min_inliers), getString(R.string.pref_default_min_inliers));
			mMaxOptimizationError = sharedPref.getString(getString(R.string.pref_key_opt_error), getString(R.string.pref_default_opt_error));
			mMaxFeatures = sharedPref.getString(getString(R.string.pref_key_features_voc), getString(R.string.pref_default_features_voc));
			String maxFeaturesLoop = sharedPref.getString(getString(R.string.pref_key_features), getString(R.string.pref_default_features));
			String featureType = sharedPref.getString(getString(R.string.pref_key_features_type), getString(R.string.pref_default_features_type));
			boolean keepAllDb = sharedPref.getBoolean(getString(R.string.pref_key_keep_all_db), Boolean.parseBoolean(getString(R.string.pref_default_keep_all_db)));
			boolean optimizeFromGraphEnd = sharedPref.getBoolean(getString(R.string.pref_key_optimize_end), Boolean.parseBoolean(getString(R.string.pref_default_optimize_end)));
			String optimizer = sharedPref.getString(getString(R.string.pref_key_optimizer), getString(R.string.pref_default_optimizer));
			
			Log.d(TAG, "set mapping parameters");
			RTABMapLib.setNodesFiltering(sharedPref.getBoolean(getString(R.string.pref_key_nodes_filtering), Boolean.parseBoolean(getString(R.string.pref_default_nodes_filtering))));
			RTABMapLib.setAutoExposure(sharedPref.getBoolean(getString(R.string.pref_key_auto_exposure), Boolean.parseBoolean(getString(R.string.pref_default_auto_exposure))));
			RTABMapLib.setRawScanSaved(sharedPref.getBoolean(getString(R.string.pref_key_raw_scan_saved), Boolean.parseBoolean(getString(R.string.pref_default_raw_scan_saved))));
			RTABMapLib.setFullResolution(sharedPref.getBoolean(getString(R.string.pref_key_resolution), Boolean.parseBoolean(getString(R.string.pref_default_resolution))));
			RTABMapLib.setAppendMode(sharedPref.getBoolean(getString(R.string.pref_key_append), Boolean.parseBoolean(getString(R.string.pref_default_append))));
			RTABMapLib.setMappingParameter("Rtabmap/DetectionRate", mUpdateRate);
			RTABMapLib.setMappingParameter("Rtabmap/TimeThr", mTimeThr);
			RTABMapLib.setMappingParameter("Rtabmap/MemoryThr", memThr);
			RTABMapLib.setMappingParameter("Mem/RehearsalSimilarity", simThr);
			RTABMapLib.setMappingParameter("Kp/MaxFeatures", mMaxFeatures);
			RTABMapLib.setMappingParameter("Vis/MaxFeatures", maxFeaturesLoop);
			RTABMapLib.setMappingParameter("Vis/MinInliers", mMinInliers);
			RTABMapLib.setMappingParameter("Rtabmap/LoopThr", mLoopThr);
			RTABMapLib.setMappingParameter("RGBD/OptimizeMaxError", mMaxOptimizationError);
			RTABMapLib.setMappingParameter("Kp/DetectorStrategy", featureType);
			RTABMapLib.setMappingParameter("Vis/FeatureType", featureType);
			RTABMapLib.setMappingParameter("Mem/NotLinkedNodesKept", String.valueOf(keepAllDb));
			RTABMapLib.setMappingParameter("RGBD/OptimizeFromGraphEnd", String.valueOf(optimizeFromGraphEnd));
			RTABMapLib.setMappingParameter("Optimizer/Strategy", optimizer);
	
			Log.d(TAG, "set exporting parameters...");
			RTABMapLib.setMeshDecimation(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_density), getString(R.string.pref_default_density))));
			RTABMapLib.setMaxCloudDepth(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_depth), getString(R.string.pref_default_depth))));
			RTABMapLib.setPointSize(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_point_size), getString(R.string.pref_default_point_size))));
			RTABMapLib.setMeshAngleTolerance(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_angle), getString(R.string.pref_default_angle))));
			RTABMapLib.setMeshTriangleSize(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_triangle), getString(R.string.pref_default_triangle))));
			
			Log.d(TAG, "set rendering parameters...");
			RTABMapLib.setMinClusterSize(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_min_cluster_size), getString(R.string.pref_default_min_cluster_size))));
			RTABMapLib.setMaxGainRadius(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_gain_max_radius), getString(R.string.pref_default_gain_max_radius))));
	
			if(mItemRenderingPointCloud != null)
			{
				int renderingType =  sharedPref.getInt(getString(R.string.pref_key_rendering), Integer.parseInt(getString(R.string.pref_default_rendering)));
				if(renderingType == 0)
				{
					mItemRenderingPointCloud.setChecked(true);
				}
				else if(renderingType == 1)
				{
					mItemRenderingMesh.setChecked(true);
				}
				else
				{
					mItemRenderingTextureMesh.setChecked(true);
				}
				RTABMapLib.setMeshRendering(
						mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked(), 
						mItemRenderingTextureMesh.isChecked());
				
				mButtonBackfaceShown.setVisibility(mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked()?View.VISIBLE:View.INVISIBLE);
			}
		}
		catch(Exception e)
		{
			Log.e(TAG, "Error parsing preferences: " + e.getMessage());
			mToast.makeText(this, String.format("Error parsing preferences: "+e.getMessage()), mToast.LENGTH_LONG).show();
		}

		Log.i(TAG, String.format("onResume()"));

		if (Tango.hasPermission(this, Tango.PERMISSIONTYPE_MOTION_TRACKING)) {

			mGLView.onResume();

		} else {
			Log.i(TAG, String.format("Asking for motion tracking permission"));
			startActivityForResult(
					Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
					Tango.TANGO_INTENT_ACTIVITYCODE);
		}
		
		TangoInitializationHelper.bindTangoService(getActivity(), mTangoServiceConnection);
	}
	
	private void setCamera(int type)
	{
		RTABMapLib.setCamera(type);
		mButtonFirst.setChecked(type==0);
		mButtonThird.setChecked(type==1);
		mButtonTop.setChecked(type==2);
	}

	@Override
	public void onClick(View v) {
		// Handle button clicks.
		switch (v.getId()) {
		case R.id.first_person_button:
			setCamera(0);

			break;
		case R.id.third_person_button:
			setCamera(1);
			break;
		case R.id.top_down_button:
			setCamera(2);
			break;
		case R.id.pause_button:
			pauseMapping();
			break;
		case R.id.light_button:
			RTABMapLib.setLighting(mButtonLighting.isChecked());
			break;
		case R.id.backface_button:
			RTABMapLib.setBackfaceCulling(!mButtonBackfaceShown.isChecked());
			break;
		case R.id.close_visualization_button:
			updateState(State.STATE_IDLE);
			break;
		case R.id.button_saveOnDevice:
			saveOnDevice();
			break;
		case R.id.button_shareToSketchfab:
			shareToSketchfab();
			break;
		default:
			return;
		}
	}
	
	private void setAndroidOrientation() {
        Display display = getWindowManager().getDefaultDisplay();
        Camera.CameraInfo colorCameraInfo = new Camera.CameraInfo();
        Camera.getCameraInfo(0, colorCameraInfo);
        RTABMapLib.setScreenRotation(display.getRotation(), colorCameraInfo.orientation);
    }

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		// Pass the touch event to the native layer for camera control.
		// Single touch to rotate the camera around the device.
		// Two fingers to zoom in and out.
		int pointCount = event.getPointerCount();
		if (pointCount == 1) {
			float normalizedX = event.getX(0) / mScreenSize.x;
			float normalizedY = event.getY(0) / mScreenSize.y;
			RTABMapLib.onTouchEvent(1, 
					event.getActionMasked(), normalizedX, normalizedY, 0.0f, 0.0f);
		}
		if (pointCount == 2) {
			if (event.getActionMasked() == MotionEvent.ACTION_POINTER_UP) {
				int index = event.getActionIndex() == 0 ? 1 : 0;
				float normalizedX = event.getX(index) / mScreenSize.x;
				float normalizedY = event.getY(index) / mScreenSize.y;
				RTABMapLib.onTouchEvent(1, 
						MotionEvent.ACTION_DOWN, normalizedX, normalizedY, 0.0f, 0.0f);
			} else {
				float normalizedX0 = event.getX(0) / mScreenSize.x;
				float normalizedY0 = event.getY(0) / mScreenSize.y;
				float normalizedX1 = event.getX(1) / mScreenSize.x;
				float normalizedY1 = event.getY(1) / mScreenSize.y;
				RTABMapLib.onTouchEvent(2, event.getActionMasked(),
						normalizedX0, normalizedY0, normalizedX1, normalizedY1);
			}
		}
		return true;
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		Log.i(TAG, "called onCreateOptionsMenu;");

		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.optionmenu, menu);
		
		getActionBar().setDisplayShowHomeEnabled(true);
		getActionBar().setIcon(R.drawable.ic_launcher);

		mItemSave = menu.findItem(R.id.save);
		mItemOpen = menu.findItem(R.id.open);
		mItemPostProcessing = menu.findItem(R.id.post_processing);
		mItemExport = menu.findItem(R.id.export);
		mItemSettings = menu.findItem(R.id.settings);
		mItemModes = menu.findItem(R.id.modes);
		mItemReset = menu.findItem(R.id.reset);
		mItemLocalizationMode = menu.findItem(R.id.localization_mode);
		mItemTrajectoryMode = menu.findItem(R.id.trajectory_mode);
		mItemRenderingPointCloud = menu.findItem(R.id.point_cloud);
		mItemRenderingMesh = menu.findItem(R.id.mesh);
		mItemRenderingTextureMesh = menu.findItem(R.id.texture_mesh);
		mItemDataRecorderMode = menu.findItem(R.id.data_recorder);
		mItemSave.setEnabled(false);
		mItemExport.setEnabled(false);
		mItemOpen.setEnabled(false);
		mItemPostProcessing.setEnabled(false);
		mItemDataRecorderMode.setEnabled(false);

		try
		{
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			int renderingType =  sharedPref.getInt(getString(R.string.pref_key_rendering), Integer.parseInt(getString(R.string.pref_default_rendering)));
			if(renderingType == 0)
			{
				mItemRenderingPointCloud.setChecked(true);
			}
			else if(renderingType == 1)
			{
				mItemRenderingMesh.setChecked(true);
			}
			else
			{
				mItemRenderingTextureMesh.setChecked(true);
			}
			RTABMapLib.setMeshRendering(
					mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked(), 
					mItemRenderingTextureMesh.isChecked());
			
			if(mButtonBackfaceShown != null)
			{
				mButtonBackfaceShown.setVisibility(mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked()?View.VISIBLE:View.INVISIBLE);
			}
		}
		catch(Exception e)
		{
			Log.e(TAG, "Error parsing rendering preferences: " + e.getMessage());
			mToast.makeText(this, String.format("Error parsing rendering preferences: "+e.getMessage()), mToast.LENGTH_LONG).show();
		}

		updateState(mState);

		return true;
	}
	
	private long getFreeMemory()
	{
		MemoryInfo mi = new MemoryInfo();
		ActivityManager activityManager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
		activityManager.getMemoryInfo(mi);
		return mi.availMem / 0x100000L; // MB
	}

	private void updateStatsUI(
			int nodes, 
			int words, 
			int points, 
			int polygons,
			float updateTime, 
			int loopClosureId,
			int highestHypId,
			int databaseMemoryUsed,
			int inliers,
			int matches,
			int featuresExtracted,
			float hypothesis,
			int nodesDrawn,
			float fps,
			int rejected,
			float rehearsalValue,
			float optimizationMaxError)
	{
		if(mButtonPause!=null)
		{
			if(mButtonPause.isChecked())
			{
				((TextView)findViewById(R.id.status)).setText("Paused");
			}
			else
			{
				String updateValue = mUpdateRate.compareTo("0")==0?"Max":mUpdateRate;
				((TextView)findViewById(R.id.status)).setText(mItemLocalizationMode.isChecked()?String.format("Localization (%s Hz)", updateValue):mItemDataRecorderMode.isChecked()?String.format("Recording (%s Hz)", updateValue):String.format("Mapping (%s Hz)", updateValue));
			}
		}

		((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
		((TextView)findViewById(R.id.free_memory)).setText(String.valueOf(getFreeMemory()));
		((TextView)findViewById(R.id.points)).setText(String.valueOf(points));
		((TextView)findViewById(R.id.polygons)).setText(String.valueOf(polygons));
		((TextView)findViewById(R.id.nodes)).setText(String.format("%d (%d shown)", nodes, nodesDrawn));
		((TextView)findViewById(R.id.words)).setText(String.valueOf(words));
		((TextView)findViewById(R.id.database_size)).setText(String.valueOf(databaseMemoryUsed));
		((TextView)findViewById(R.id.inliers)).setText(String.valueOf(inliers));
		((TextView)findViewById(R.id.features)).setText(String.format("%d / %s", featuresExtracted, mMaxFeatures.compareTo("0")==0?"No Limit":mMaxFeatures.compareTo("-1")==0?"Disabled":mMaxFeatures));
		((TextView)findViewById(R.id.rehearsal)).setText(String.format("%.3f", rehearsalValue));
		((TextView)findViewById(R.id.update_time)).setText(String.format("%.3f / %s", updateTime, mTimeThr.compareTo("0")==0?"No Limit":mTimeThr));
		((TextView)findViewById(R.id.hypothesis)).setText(String.format("%.3f / %s (%d)", hypothesis, mLoopThr, loopClosureId>0?loopClosureId:highestHypId));
		((TextView)findViewById(R.id.fps)).setText(String.format("%.3f Hz", fps));
		if(mButtonPause!=null && !mButtonPause.isChecked())
		{
			if(loopClosureId > 0)
			{
				++mTotalLoopClosures;

				mToast.setText(String.format("Loop closure detected! (%d/%d inliers)", inliers, matches));
				mToast.show();
			}
			else if(rejected > 0)
			{
				if(inliers >= Integer.parseInt(mMinInliers))
				{
					mToast.setText(String.format("Loop closure rejected, too high graph optimization error (%.3fm > %sm).", optimizationMaxError, mMaxOptimizationError));
				}
				else
				{
					mToast.setText(String.format("Loop closure rejected, not enough inliers (%d/%d < %s).", inliers, matches, mMinInliers));
				}
				mToast.show();
			}
		}
		((TextView)findViewById(R.id.total_loop)).setText(String.valueOf(mTotalLoopClosures));
	}

	// called from jni
	public void updateStatsCallback(
			final int nodes, 
			final int words, 
			final int points, 
			final int polygons,
			final float updateTime, 
			final int loopClosureId,
			final int highestHypId,
			final int databaseMemoryUsed,
			final int inliers,
			final int matches,
			final int features,
			final float hypothesis,
			final int nodesDrawn,
			final float fps,
			final int rejected,
			final float rehearsalValue,
			final float optimizationMaxError)
	{
		Log.i(TAG, String.format("updateStatsCallback()"));

		runOnUiThread(new Runnable() {
			public void run() {
				updateStatsUI(nodes, words, points, polygons, updateTime, loopClosureId, highestHypId, databaseMemoryUsed, inliers, matches, features, hypothesis, nodesDrawn, fps, rejected, rehearsalValue, optimizationMaxError);
			} 
		});
	}

	private void rtabmapInitEventUI(
			int status, 
			String msg)
	{
		Log.i(TAG, String.format("rtabmapInitEventsUI() status=%d msg=%s", status, msg));

		if(mButtonPause!=null)
		{
			if(mButtonPause.isChecked())
			{
				((TextView)findViewById(R.id.status)).setText(
						status == 1 && msg.isEmpty()?"Paused":msg);
			}
			else
			{
				((TextView)findViewById(R.id.status)).setText(
						status == 1 && msg.isEmpty()?(mItemLocalizationMode!=null&&mItemLocalizationMode.isChecked()?"Localization":mItemDataRecorderMode!=null&&mItemDataRecorderMode.isChecked()?"Recording":"Mapping"):msg);
			}
		}
	}

	//called from jni
	public void rtabmapInitEventCallback(
			final int status, 
			final String msg)
	{
		Log.i(TAG, String.format("rtabmapInitEventCallback()"));

		runOnUiThread(new Runnable() {
			public void run() {
				rtabmapInitEventUI(status, msg);
			} 
		});
	}

	private void tangoEventUI(
			int type, 
			String key,
			String value)
	{
		/**
		 * 
		 * "TangoServiceException:X" - The service has encountered an exception, and a text description is given in X.
		 * "FisheyeOverExposed:X" - the fisheye image is over exposed with average pixel value X px.
		 * "FisheyeUnderExposed:X" - the fisheye image is under exposed with average pixel value X px.
		 * "ColorOverExposed:X" - the color image is over exposed with average pixel value X px.
		 * "ColorUnderExposed:X" - the color image is under exposed with average pixel value X px.
		 * "TooFewFeaturesTracked:X" - too few features were tracked in the fisheye image. The number of features tracked is X.
		 * "AreaDescriptionSaveProgress:X" - ADF saving is X * 100 percent complete.
		 * "Unknown"
		 */
		String str = null;
		if(key.equals("TangoServiceException"))
			str = String.format("Tango service exception: %s", value);
		else if(key.equals("FisheyeOverExposed"))
			;//str = String.format("The fisheye image is over exposed with average pixel value %s px.", value);
		else if(key.equals("FisheyeUnderExposed"))
			;//str = String.format("The fisheye image is under exposed with average pixel value %s px.", value);
		else if(key.equals("ColorOverExposed")) 
			;//str = String.format("The color image is over exposed with average pixel value %s px.", value);
		else if(key.equals("ColorUnderExposed")) 
			;//str = String.format("The color image is under exposed with average pixel value %s px.", value);
		else if(key.equals("CameraTango")) 
			str = value;
		else if(key.equals("TooFewFeaturesTracked")) 
		{
			if(!value.equals("0"))
			{
				str = String.format("Too few features (%s) were tracked in the fisheye image. This may result in poor odometry!", value);
			}
		}
		else	
		{
			str = String.format("Unknown Tango event detected!? (type=%d)", type);
		}
		if(str!=null)
		{
			mToast.setText(str);
			mToast.show();
		}
	}

	//called from jni
	public void tangoEventCallback(
			final int type, 
			final String key,
			final String value)
	{
		if(mButtonPause != null && !mButtonPause.isChecked())
		{
			runOnUiThread(new Runnable() {
				public void run() {
					tangoEventUI(type, key, value);
				} 
			});
		}
	}

	private boolean CheckTangoCoreVersion(int minVersion) {
		int versionNumber = 0;
		String packageName = TANGO_PACKAGE_NAME;
		try {
			PackageInfo pi = getApplicationContext().getPackageManager().getPackageInfo(packageName,
					PackageManager.GET_META_DATA);
			versionNumber = pi.versionCode;
		} catch (NameNotFoundException e) {
			e.printStackTrace();
		}
		return (minVersion <= versionNumber);
	}

	private RTABMapActivity getActivity() {return this;}

	private String[] loadFileList(String directory) {
		File path = new File(directory); 
		String fileList[];
		try {
			path.mkdirs();
		}
		catch(SecurityException e) {
			Log.e(TAG, "unable to write on the sd card " + e.toString());
		}
		if(path.exists()) {
			FilenameFilter filter = new FilenameFilter() {

				@Override
				public boolean accept(File dir, String filename) {
					File sel = new File(dir, filename);
					return filename.compareTo(RTABMAP_TMP_DB) != 0 && filename.endsWith(".db");
				}

			};
			fileList = path.list(filter);
			Arrays.sort(fileList);
		}
		else {
			fileList = new String[0];
		}
		return fileList;
	}

	private void standardOptimization() {
		mProgressDialog.setTitle("Post-Processing");
		mProgressDialog.setMessage(String.format("Please wait while optimizing..."));
		mProgressDialog.show();

		updateState(State.STATE_PROCESSING);
		Thread workingThread = new Thread(new Runnable() {
			public void run() {
				final int loopDetected = RTABMapLib.postProcessing(-1);
				runOnUiThread(new Runnable() {
					public void run() {
						if(loopDetected >= 0)
						{
							mTotalLoopClosures+=loopDetected;
							mProgressDialog.setMessage(String.format("Optimization done! Increasing visual appeal..."));
						}
						else if(loopDetected < 0)
						{
							mToast.makeText(getActivity(), String.format("Optimization failed!"), mToast.LENGTH_SHORT).show();
						}
						updateState(State.STATE_IDLE);
					}
				});
			} 
		});
		workingThread.start();
	}

	private void updateState(State state)
	{	
		mState = state;
		switch(state)
		{
		case STATE_PROCESSING:
			mButtonLighting.setVisibility(View.INVISIBLE);
			mButtonCloseVisualization.setVisibility(View.INVISIBLE);
			mButtonSaveOnDevice.setVisibility(View.INVISIBLE);
			mButtonShareOnSketchfab.setVisibility(View.INVISIBLE);
			mItemSave.setEnabled(false);
			mItemExport.setEnabled(false);
			mItemOpen.setEnabled(false);
			mItemPostProcessing.setEnabled(false);
			mItemSettings.setEnabled(false);
			mItemReset.setEnabled(false);
			mItemModes.setEnabled(false);
			mButtonPause.setVisibility(View.INVISIBLE);
			if(mMemStatusUpdateThread == null)
			{
				mMemStatusUpdateThread = new Thread() {
		
		        	@Override
		        	public void run() {
		        		try {
		        			while (!isInterrupted()) {
		        				Thread.sleep(1000);
		        				if(mState == RTABMapActivity.State.STATE_PROCESSING)
		        				{
			        				runOnUiThread(new Runnable() {
			        					@Override
			        					public void run() {
			        						((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
			        						((TextView)findViewById(R.id.free_memory)).setText(String.valueOf(getFreeMemory()));
			        					}
			        				});
		        				}
		        			}
		        		} catch (InterruptedException e) {
		        		}
		        	}
		        };
		        mMemStatusUpdateThread.start();
			}
			break;
		case STATE_VISUALIZING:
			mButtonLighting.setVisibility(View.VISIBLE);
			mButtonCloseVisualization.setVisibility(View.VISIBLE);
			mButtonSaveOnDevice.setVisibility(View.VISIBLE);
			mButtonShareOnSketchfab.setVisibility(View.VISIBLE);
			mItemSave.setEnabled(mButtonPause.isChecked());
			mItemExport.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemOpen.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemPostProcessing.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemSettings.setEnabled(true);
			mItemReset.setEnabled(true);
			mItemModes.setEnabled(true);
			mButtonPause.setVisibility(View.INVISIBLE);
			mItemDataRecorderMode.setEnabled(mButtonPause.isChecked());
			if(mMemStatusUpdateThread != null)
			{
				Thread tmp = mMemStatusUpdateThread;
				mMemStatusUpdateThread = null;
				tmp.interrupt();
			}
			break;
		default:
			mButtonLighting.setVisibility(View.INVISIBLE);
			mButtonCloseVisualization.setVisibility(View.INVISIBLE);
			mButtonSaveOnDevice.setVisibility(View.INVISIBLE);
			mButtonShareOnSketchfab.setVisibility(View.INVISIBLE);
			mItemSave.setEnabled(mButtonPause.isChecked());
			mItemExport.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemOpen.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemPostProcessing.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemSettings.setEnabled(true);
			mItemReset.setEnabled(true);
			mItemModes.setEnabled(true);
			mButtonPause.setVisibility(View.VISIBLE);
			mItemDataRecorderMode.setEnabled(mButtonPause.isChecked());
			RTABMapLib.postExportation(false);
			if(mMemStatusUpdateThread != null)
			{
				Thread tmp = mMemStatusUpdateThread;
				mMemStatusUpdateThread = null;
				tmp.interrupt();
			}
			break;
		}
	}

	private void pauseMapping() {

		updateState(State.STATE_IDLE);

		if(mButtonPause.isChecked())
		{
			RTABMapLib.setPausedMapping(true);
			((TextView)findViewById(R.id.status)).setText("Paused");
			mMapIsEmpty = false;
			mDateOnPause = new Date();

			if(!mOnPause && !mItemLocalizationMode.isChecked() && !mItemDataRecorderMode.isChecked())
			{
				// Do standard post processing?
				new AlertDialog.Builder(getActivity())
				.setTitle("Mapping Paused! Optimize Now?")
				.setMessage("Do you want to do standard map optimization now? This can be also done later using \"Optimize\" menu.")
				.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int which) {
						standardOptimization();
					}
				})
				.setNegativeButton("No", new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int which) {
						// do nothing...
					}
				})
				.show();
			} 
		}
		else
		{
			RTABMapLib.setPausedMapping(false);
			((TextView)findViewById(R.id.status)).setText(mItemLocalizationMode.isChecked()?"Localization":mItemDataRecorderMode.isChecked()?"Recording":"Mapping");
			if(mItemDataRecorderMode.isChecked())
			{
				mToast.makeText(getActivity(), String.format("Data Recorder Mode: no map is created, only raw data is recorded."), mToast.LENGTH_LONG).show();
			}
			else if(!mMapIsEmpty)
			{
				mToast.makeText(getActivity(), String.format("On resume, a new map is created. Tip: Try relocalizing in the previous area."), mToast.LENGTH_LONG).show();
			}
		}
	}

	public boolean onOptionsItemSelected(MenuItem item) {
		Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
		int itemId = item.getItemId();
		if (itemId == R.id.post_processing_standard)
		{
			standardOptimization();
		}
		else if (itemId == R.id.detect_more_loop_closures)
		{
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Please wait while detecting more loop closures..."));
			mProgressDialog.show();
			updateState(State.STATE_PROCESSING);
			Thread workingThread = new Thread(new Runnable() {
				public void run() {
					final int loopDetected = RTABMapLib.postProcessing(2);
					runOnUiThread(new Runnable() {
						public void run() {
							mProgressDialog.dismiss();
							if(loopDetected >= 0)
							{
								mTotalLoopClosures+=loopDetected;
								mToast.makeText(getActivity(), String.format("Detection done! %d new loop closure(s) added.", loopDetected), mToast.LENGTH_SHORT).show();
							}
							else if(loopDetected < 0)
							{
								mToast.makeText(getActivity(), String.format("Detection failed!"), mToast.LENGTH_SHORT).show();
							}
							updateState(State.STATE_IDLE);
						}
					});
				} 
			});
			workingThread.start();
		}
		else if (itemId == R.id.global_graph_optimization)
		{
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Global graph optimization..."));
			mProgressDialog.show();
			updateState(State.STATE_PROCESSING);
			Thread workingThread = new Thread(new Runnable() {
				public void run() {
					final int value = RTABMapLib.postProcessing(0);
					runOnUiThread(new Runnable() {
						public void run() {
							mProgressDialog.dismiss();
							if(value >= 0)
							{
								mToast.makeText(getActivity(), String.format("Optimization done!"), mToast.LENGTH_SHORT).show();
							}
							else if(value < 0)
							{
								mToast.makeText(getActivity(), String.format("Optimization failed!"), mToast.LENGTH_SHORT).show();
							}
							updateState(State.STATE_IDLE);
						}
					});
				} 
			});
			workingThread.start();
		}
		else if (itemId == R.id.polygons_filtering)
		{		
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Noise filtering..."));
			mProgressDialog.show();
			RTABMapLib.postProcessing(4);
		}
		else if (itemId == R.id.gain_compensation_fast)
		{		
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Fast gain compensation..."));
			mProgressDialog.show();
			RTABMapLib.postProcessing(5);
		}
		else if (itemId == R.id.gain_compensation_full)
		{		
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Full gain compensation..."));
			mProgressDialog.show();
			RTABMapLib.postProcessing(6);
		}
		else if (itemId == R.id.bilateral_filtering)
		{		
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Mesh smoothing..."));
			mProgressDialog.show();
			RTABMapLib.postProcessing(7);
		}
		else if (itemId == R.id.sba)
		{
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Bundle adjustement..."));
			mProgressDialog.show();

			Thread workingThread = new Thread(new Runnable() {
				public void run() {
					final int value = RTABMapLib.postProcessing(1);
					runOnUiThread(new Runnable() {
						public void run() {
							mProgressDialog.dismiss();
							if(value >= 0)
							{
								mToast.makeText(getActivity(), String.format("Optimization done!"), mToast.LENGTH_SHORT).show();
							}
							else if(value < 0)
							{
								mToast.makeText(getActivity(), String.format("Optimization failed!"), mToast.LENGTH_SHORT).show();
							}
						}
					});
				} 
			});
			workingThread.start();
		}
		else if(itemId == R.id.debug)
		{
			item.setChecked(!item.isChecked());
			if(!item.isChecked())
			{
				mLayoutDebug.setVisibility(LinearLayout.GONE);
			}
			else
			{
				mLayoutDebug.setVisibility(LinearLayout.VISIBLE);
			}
		}
		else if(itemId == R.id.mesh || itemId == R.id.texture_mesh || itemId == R.id.point_cloud)
		{
			item.setChecked(true);
			RTABMapLib.setMeshRendering(
					mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked(), 
					mItemRenderingTextureMesh.isChecked());
			
			mButtonBackfaceShown.setVisibility(mItemRenderingMesh.isChecked() || mItemRenderingTextureMesh.isChecked()?View.VISIBLE:View.INVISIBLE);

			// save preference
			int type = mItemRenderingPointCloud.isChecked()?0:mItemRenderingMesh.isChecked()?1:2;
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			SharedPreferences.Editor editor = sharedPref.edit();
			editor.putInt(getString(R.string.pref_key_rendering), type);
			// Commit the edits!
			editor.commit();
		}
		else if(itemId == R.id.map_shown)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setMapCloudShown(item.isChecked());
		}
		else if(itemId == R.id.odom_shown)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setOdomCloudShown(item.isChecked());
		}
		else if(itemId == R.id.localization_mode)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setLocalizationMode(item.isChecked());
		}
		else if(itemId == R.id.trajectory_mode)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setTrajectoryMode(item.isChecked());
			setCamera(item.isChecked()?2:1);
		}
		else if(itemId == R.id.graph_optimization)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setGraphOptimization(item.isChecked());
		}
		else if(itemId == R.id.graph_visible)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setGraphVisible(item.isChecked());
		}
		else if(itemId == R.id.grid_visible)
		{
			item.setChecked(!item.isChecked());
			RTABMapLib.setGridVisible(item.isChecked());
		}
		else if (itemId == R.id.save)
		{
			AlertDialog.Builder builder = new AlertDialog.Builder(this);
			builder.setTitle("RTAB-Map Database Name (*.db):");
			final EditText input = new EditText(this);
			input.setInputType(InputType.TYPE_CLASS_TEXT); 
			if(mOpenedDatabasePath.isEmpty())
			{
				String timeStamp = new SimpleDateFormat("yyMMdd-HHmmss").format(mDateOnPause);
				input.setText(timeStamp);
			}
			else
			{
				File f = new File(mOpenedDatabasePath);
				String name = f.getName();
				input.setText(name.substring(0,name.lastIndexOf(".")));
			}
			input.setImeOptions(EditorInfo.IME_FLAG_NO_EXTRACT_UI);
			input.setSelectAllOnFocus(true);
			input.selectAll();
			builder.setView(input);
			builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
				@Override
				public void onClick(DialogInterface dialog, int which)
				{
					final String fileName = input.getText().toString();  
					dialog.dismiss();
					if(!fileName.isEmpty())
					{
						File newFile = new File(mWorkingDirectory + fileName + ".db");
						if(newFile.exists())
						{
							new AlertDialog.Builder(getActivity())
							.setTitle("File Already Exists")
							.setMessage("Do you want to overwrite the existing file?")
							.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
								public void onClick(DialogInterface dialog, int which) {

									final String newDatabasePath = mWorkingDirectory + fileName + ".db";
									mProgressDialog.setTitle("Saving");
									if(mOpenedDatabasePath.equals(newDatabasePath))
									{
										mProgressDialog.setMessage(String.format("Please wait while updating \"%s\"...", newDatabasePath));
									}
									else
									{
										mProgressDialog.setMessage(String.format("Please wait while saving \"%s\"...", newDatabasePath));
									}
									mProgressDialog.show();
									updateState(State.STATE_PROCESSING);
									Thread saveThread = new Thread(new Runnable() {
										public void run() {
											RTABMapLib.save(newDatabasePath); // save
											runOnUiThread(new Runnable() {
												public void run() {
													if(mOpenedDatabasePath.equals(newDatabasePath))
													{
														mToast.makeText(getActivity(), String.format("Database \"%s\" updated.", newDatabasePath), mToast.LENGTH_LONG).show();
													}
													else
													{
														mToast.makeText(getActivity(), String.format("Database saved to \"%s\".", newDatabasePath), mToast.LENGTH_LONG).show();

														Intent intent = new Intent(getActivity(), RTABMapActivity.class);
														// use System.currentTimeMillis() to have a unique ID for the pending intent
														PendingIntent pIntent = PendingIntent.getActivity(getActivity(), (int) System.currentTimeMillis(), intent, 0);

														// build notification
														// the addAction re-use the same intent to keep the example short
														Notification n  = new Notification.Builder(getActivity())
														.setContentTitle(getString(R.string.app_name))
														.setContentText(newDatabasePath + " saved!")
														.setSmallIcon(R.drawable.ic_launcher)
														.setContentIntent(pIntent)
														.setAutoCancel(true).build();


														NotificationManager notificationManager = 
																(NotificationManager) getSystemService(NOTIFICATION_SERVICE);

														notificationManager.notify(0, n); 
													}
													if(!mItemDataRecorderMode.isChecked())
													{
														mOpenedDatabasePath = newDatabasePath;
													}
													mProgressDialog.dismiss();
													updateState(State.STATE_IDLE);
												}
											});
										} 
									});
									saveThread.start();
								}
							})
							.setNegativeButton("No", new DialogInterface.OnClickListener() {
								public void onClick(DialogInterface dialog, int which) {
									dialog.dismiss();
								}
							})
							.show();
						}
						else
						{
							final String newDatabasePath = mWorkingDirectory + fileName + ".db";
							mProgressDialog.setTitle("Saving");
							if(mOpenedDatabasePath.equals(newDatabasePath))
							{
								mProgressDialog.setMessage(String.format("Please wait while updating \"%s\"...", mOpenedDatabasePath));
							}
							else
							{
								mProgressDialog.setMessage(String.format("Please wait while saving \"%s\"...", newDatabasePath));
							}
							mProgressDialog.show();
							updateState(State.STATE_PROCESSING);
							Thread saveThread = new Thread(new Runnable() {
								public void run() {
									RTABMapLib.save(newDatabasePath); // save
									runOnUiThread(new Runnable() {
										public void run() {
											if(mOpenedDatabasePath.equals(newDatabasePath))
											{
												mToast.makeText(getActivity(), String.format("Database \"%s\" updated.", newDatabasePath), mToast.LENGTH_LONG).show();
											}
											else
											{
												mToast.makeText(getActivity(), String.format("Database saved to \"%s\".", newDatabasePath), mToast.LENGTH_LONG).show();

												Intent intent = new Intent(getActivity(), RTABMapActivity.class);
												// use System.currentTimeMillis() to have a unique ID for the pending intent
												PendingIntent pIntent = PendingIntent.getActivity(getActivity(), (int) System.currentTimeMillis(), intent, 0);

												// build notification
												// the addAction re-use the same intent to keep the example short
												Notification n  = new Notification.Builder(getActivity())
												.setContentTitle(getString(R.string.app_name))
												.setContentText(newDatabasePath + " saved!")
												.setSmallIcon(R.drawable.ic_launcher)
												.setContentIntent(pIntent)
												.setAutoCancel(true).build();


												NotificationManager notificationManager = 
														(NotificationManager) getSystemService(NOTIFICATION_SERVICE);

												notificationManager.notify(0, n); 
											}
											if(!mItemDataRecorderMode.isChecked())
											{
												mOpenedDatabasePath = newDatabasePath;
											}
											mProgressDialog.dismiss();
											updateState(State.STATE_IDLE);
										}
									});
								} 
							});
							saveThread.start();
						}
					}
				}
			});
			AlertDialog alertToShow = builder.create();
			alertToShow.getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE);
			alertToShow.show();
		}
		else if(itemId == R.id.reset)
		{
			((TextView)findViewById(R.id.points)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.polygons)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.nodes)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.words)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.inliers)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.features)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.update_time)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.hypothesis)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.fps)).setText(String.valueOf(0));
			mTotalLoopClosures = 0;
			((TextView)findViewById(R.id.total_loop)).setText(String.valueOf(mTotalLoopClosures));

			mOpenedDatabasePath = "";
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			boolean databaseInMemory = sharedPref.getBoolean(getString(R.string.pref_key_db_in_memory), Boolean.parseBoolean(getString(R.string.pref_default_db_in_memory)));
			String tmpDatabase = mWorkingDirectory+RTABMAP_TMP_DB;
			(new File(tmpDatabase)).delete();
			RTABMapLib.openDatabase(tmpDatabase, databaseInMemory, false);
			
			mMapIsEmpty = true;
			mItemSave.setEnabled(false);
			mItemExport.setEnabled(false);
			mItemPostProcessing.setEnabled(false);
			updateState(State.STATE_IDLE);
		}
		else if(itemId == R.id.data_recorder)
		{
			final boolean dataRecorderOldState = item.isChecked();
			new AlertDialog.Builder(getActivity())
			.setTitle("Data Recorder Mode")
			.setMessage("Changing from/to data recorder mode will close the current session. Do you want to continue?")
			.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {           	  
					// reset
					((TextView)findViewById(R.id.points)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.polygons)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.nodes)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.words)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.inliers)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.features)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.update_time)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.hypothesis)).setText(String.valueOf(0));
					((TextView)findViewById(R.id.fps)).setText(String.valueOf(0));
					mTotalLoopClosures = 0;
					((TextView)findViewById(R.id.total_loop)).setText(String.valueOf(mTotalLoopClosures));

					mItemDataRecorderMode.setChecked(!dataRecorderOldState);
					RTABMapLib.setDataRecorderMode(mItemDataRecorderMode.isChecked());

					mOpenedDatabasePath = "";
					SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getActivity());
					boolean databaseInMemory = sharedPref.getBoolean(getString(R.string.pref_key_db_in_memory), Boolean.parseBoolean(getString(R.string.pref_default_db_in_memory)));
					String tmpDatabase = mWorkingDirectory+RTABMAP_TMP_DB;
					(new File(tmpDatabase)).delete();
					RTABMapLib.openDatabase(tmpDatabase, databaseInMemory, false);

					mItemOpen.setEnabled(!mItemDataRecorderMode.isChecked() && mButtonPause.isChecked());
					mItemPostProcessing.setEnabled(!mItemDataRecorderMode.isChecked() && mButtonPause.isChecked());
					mItemExport.setEnabled(!mItemDataRecorderMode.isChecked() && mButtonPause.isChecked());

					mItemLocalizationMode.setEnabled(!mItemDataRecorderMode.isChecked());		   

					if(mItemDataRecorderMode.isChecked())
					{
						mToast.makeText(getActivity(), String.format("Data recorder mode activated! Tip: You can increase data update rate in Parameters menu under Mapping options."), mToast.LENGTH_LONG).show();
					}
					else
					{
						mToast.makeText(getActivity(), String.format("Data recorder mode deactivated!"), mToast.LENGTH_LONG).show();
					}
				}
			})
			.setNegativeButton("No", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {
					dialog.dismiss();
				}
			})
			.show();
		}
		else if(itemId == R.id.export_point_cloud || 
				itemId == R.id.export_mesh ||
				itemId == R.id.export_mesh_texture ||
				itemId == R.id.export_optimized_mesh ||
				itemId == R.id.export_optimized_mesh_texture)
		{
			final boolean isOBJ = itemId == R.id.export_mesh_texture || itemId == R.id.export_optimized_mesh_texture;
			final String extension = isOBJ? ".obj" : ".ply";

			final int polygons = Integer.parseInt(((TextView)findViewById(R.id.polygons)).getText().toString());

			final boolean meshing = itemId != R.id.export_point_cloud;
			final boolean optimized = itemId == R.id.export_optimized_mesh || itemId == R.id.export_optimized_mesh_texture;

			// get Export settings
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			final float cloudVoxelSize = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_cloud_voxel), getString(R.string.pref_default_cloud_voxel)));
			final int textureSize = isOBJ?Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_texture_size), getString(R.string.pref_default_texture_size))):0;
			final int normalK = Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_normal_k), getString(R.string.pref_default_normal_k)));
			final float maxTextureDistance = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_max_texture_distance), getString(R.string.pref_default_max_texture_distance)));
			final float optimizedVoxelSize = cloudVoxelSize;
			final int optimizedDepth = Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_opt_depth), getString(R.string.pref_default_opt_depth)));
			final float optimizedDecimationFactor = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_opt_decimation_factor), getString(R.string.pref_default_opt_decimation_factor)))/100.0f;
			final float optimizedColorRadius = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_opt_color_radius), getString(R.string.pref_default_opt_color_radius)));
			final boolean optimizedCleanWhitePolygons = sharedPref.getBoolean(getString(R.string.pref_key_opt_clean_white), Boolean.parseBoolean(getString(R.string.pref_default_opt_clean_white)));
			final boolean optimizedColorWhitePolygons = false;//sharedPref.getBoolean("pref_key_opt_color_white", false); // not used
			final boolean blockRendering = sharedPref.getBoolean(getString(R.string.pref_key_block_render), Boolean.parseBoolean(getString(R.string.pref_default_block_render)));

			mProgressDialog.setTitle("Exporting");
			mProgressDialog.setMessage(String.format("Please wait while preparing data to export..."));

			mProgressDialog.show();
			updateState(State.STATE_PROCESSING);
			final String tmpPath = mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + extension;
		
			File tmpDir = new File(mWorkingDirectory + RTABMAP_TMP_DIR);
			tmpDir.mkdirs();
			
			Thread exportThread = new Thread(new Runnable() {
				public void run() {

					final boolean success = RTABMapLib.exportMesh(
							tmpPath,
							cloudVoxelSize,
							meshing,
							textureSize,
							normalK,
							maxTextureDistance,
							optimized,
							optimizedVoxelSize,
							optimizedDepth,
							optimizedDecimationFactor,
							optimizedColorRadius,
							optimizedCleanWhitePolygons,
							optimizedColorWhitePolygons,
							blockRendering);
					runOnUiThread(new Runnable() {
						public void run() {
							if(success)
							{
								// Visualize the result?
								new AlertDialog.Builder(getActivity())
								.setCancelable(false)
								.setTitle("Export Successful!")
								.setMessage("Do you want visualize the result before saving to file or sharing to Sketchfab?")
								.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
									public void onClick(DialogInterface dialog, int which) {
										mExportedOBJ = isOBJ;
										updateState(State.STATE_VISUALIZING);
										RTABMapLib.postExportation(true);
										if(mButtonFirst.isChecked())
										{
											setCamera(2);
										}
									}
								})
								.setNegativeButton("No", new DialogInterface.OnClickListener() {
									public void onClick(DialogInterface dialog, int which) {
										mExportedOBJ = isOBJ;
										updateState(State.STATE_IDLE);
										RTABMapLib.postExportation(false);
										
										new AlertDialog.Builder(getActivity())
										.setCancelable(false)
										.setTitle("Save to...")
										.setMessage("Do you want to share to Sketchfab or save it on the device?")
										.setPositiveButton("Share to Sketchfab", new DialogInterface.OnClickListener() {
											public void onClick(DialogInterface dialog, int which) {
												shareToSketchfab();
											}
										})
										.setNegativeButton("Save on device", new DialogInterface.OnClickListener() {
											public void onClick(DialogInterface dialog, int which) {
												saveOnDevice();
											}
										})
										.show();
									}
								})
								.show();
							}
							else
							{
								updateState(State.STATE_IDLE);
								mToast.makeText(getActivity(), String.format("Exporting map failed!"), mToast.LENGTH_LONG).show();
							}
							mProgressDialog.dismiss();
						}
					});
				} 
			});
			exportThread.start();
		}
		else if(itemId == R.id.open)
		{
			final String[] files = loadFileList(mWorkingDirectory);
			if(files.length > 0)
			{
				String[] filesWithSize = new String[files.length];
				for(int i = 0; i<filesWithSize.length; ++i)
				{
					File filePath = new File(mWorkingDirectory+files[i]);
					long mb = filePath.length()/(1024*1024);
					filesWithSize[i] = files[i] + " ("+mb+" MB)";
				}
				AlertDialog.Builder builder = new AlertDialog.Builder(this);
				builder.setTitle("Choose Your File (*.db)");
				builder.setItems(filesWithSize, new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, final int which) {
						
						// Smooth and adjust color now?
						new AlertDialog.Builder(getActivity())
						.setTitle("Opening database...")
						.setMessage("Do you want to smooth and adjust colors now?\nThis can be done later under Optimize menu.")
						.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
							public void onClick(DialogInterface dialog, int whichIn) {
								openDatabase(files[which], true);
							}
						})
						.setNeutralButton("No", new DialogInterface.OnClickListener() {
							public void onClick(DialogInterface dialog, int whichIn) {
								openDatabase(files[which], false);
							}
						})
						.show();
						return;
					}
				});
				builder.show();
			}   	
		}
		else if(itemId == R.id.settings)
		{
			Intent intent = new Intent(getActivity(), SettingsActivity.class);
			startActivity(intent);
		}
		else if(itemId == R.id.about)
		{
			AboutDialog about = new AboutDialog(this);
			about.setTitle("About RTAB-Map");
			about.show();
		}

		return true;
	}
	
	private void saveOnDevice()
	{
		AlertDialog.Builder builder = new AlertDialog.Builder(this);
		final String extension  = mExportedOBJ?".obj":".ply";
		builder.setTitle(String.format("File Name (*%s):", extension));
		final EditText input = new EditText(this);
		input.setInputType(InputType.TYPE_CLASS_TEXT);        
		builder.setView(input);
		if(mOpenedDatabasePath.isEmpty())
		{
			String timeStamp = new SimpleDateFormat("yyMMdd-HHmmss").format(mDateOnPause);
			input.setText(timeStamp);
		}
		else
		{
			File f = new File(mOpenedDatabasePath);
			String name = f.getName();
			input.setText(name.substring(0,name.lastIndexOf(".")));
		}
		input.setImeOptions(EditorInfo.IME_FLAG_NO_EXTRACT_UI);
		input.setSelectAllOnFocus(true);
		input.selectAll();
		builder.setCancelable(false);
		builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which)
			{
				dialog.dismiss();
			}
		});
		builder.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which)
			{
				final String fileName = input.getText().toString();    
				dialog.dismiss();
				if(!fileName.isEmpty())
				{
					File newFile = new File(mWorkingDirectory + fileName + extension);
					if(newFile.exists())
					{
						new AlertDialog.Builder(getActivity())
						.setTitle("File Already Exists")
						.setMessage("Do you want to overwrite the existing file?")
						.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
							public void onClick(DialogInterface dialog, int which) {
								writeExportedFiles(fileName, mExportedOBJ);
							}
						})
						.setNegativeButton("No", new DialogInterface.OnClickListener() {
							public void onClick(DialogInterface dialog, int which) {
								saveOnDevice();
							}
						})
						.show();
					}
					else
					{
						writeExportedFiles(fileName, mExportedOBJ);
					}
				}
			}
		});
		AlertDialog alertToShow = builder.create();
		alertToShow.getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE);
		alertToShow.show();
	}
	
	private void writeExportedFiles(String fileName, boolean isOBJ)
	{
		final String extension  = mExportedOBJ?".obj":".ply";
		final String path = mWorkingDirectory + fileName + extension;
		
		boolean success = true;
		if(mExportedOBJ)
		{	
			File toOBJFile = new File(mWorkingDirectory + fileName + extension);
			File toMTLFile = new File(mWorkingDirectory + fileName + ".mtl");
			File toJPGFile = new File(mWorkingDirectory + fileName + ".jpg");
			
			toOBJFile.delete();
			toMTLFile.delete();
			toJPGFile.delete();
			
			File fromOBJFile = new File(mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + extension);
			File fromMTLFile = new File(mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".mtl");
			File fromJPGFile = new File(mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".jpg");
			
			try
			{
				copy(fromOBJFile,toOBJFile);
				copy(fromMTLFile,toMTLFile);
				copy(fromJPGFile,toJPGFile);
				mToast.makeText(getActivity(), String.format("Mesh \"%s\" (with texture \"%s\" and \"%s\") successfully exported!", path, fileName + ".jpg", fileName + ".mtl"), mToast.LENGTH_LONG).show();
			}
			catch(IOException e)
			{
				mToast.makeText(getActivity(), String.format("Exporting mesh \"%s\" (with texture \"%s\" and \"%s\") failed! Error=%s", path, fileName + ".jpg", fileName + ".mtl", e.getMessage()), mToast.LENGTH_LONG).show();
				success = false;
			}
		}
		else
		{
			File toPLYFile = new File(mWorkingDirectory + fileName + extension);
			toPLYFile.delete();
			File fromPLYFile = new File(mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + extension);
			
			try
			{
				copy(fromPLYFile,toPLYFile);
				mToast.makeText(getActivity(), String.format("Mesh/point cloud \"%s\" successfully exported!", path), mToast.LENGTH_LONG).show();
			}
			catch(Exception e)
			{
				mToast.makeText(getActivity(), String.format("Exporting mesh/point cloud \"%s\" failed! Error=%s", path, e.getMessage()), mToast.LENGTH_LONG).show();
				success=false;
			}
		}
		
		if(success)
		{
			Intent intent = new Intent(getActivity(), RTABMapActivity.class);
			// use System.currentTimeMillis() to have a unique ID for the pending intent
			PendingIntent pIntent = PendingIntent.getActivity(getActivity(), (int) System.currentTimeMillis(), intent, 0);

			// build notification
			// the addAction re-use the same intent to keep the example short
			Notification n  = new Notification.Builder(getActivity())
			.setContentTitle(getString(R.string.app_name))
			.setContentText(path + " exported!")
			.setSmallIcon(R.drawable.ic_launcher)
			.setContentIntent(pIntent)
			.setAutoCancel(true).build();


			NotificationManager notificationManager = 
					(NotificationManager) getSystemService(NOTIFICATION_SERVICE);

			notificationManager.notify(0, n); 
		}
	}
	
	private void openDatabase(String fileName, boolean optimize)
	{
		mOpenedDatabasePath = mWorkingDirectory + fileName;

		if(!mItemTrajectoryMode.isChecked())
		{
			mProgressDialog.setTitle("Loading");
			mProgressDialog.setMessage(String.format("Database \"%s\" loaded. Please wait while creating point clouds and meshes...", fileName));
			mProgressDialog.show();
			updateState(State.STATE_PROCESSING);
		}
		
		String tmpDatabase = mWorkingDirectory+RTABMAP_TMP_DB;
		(new File(tmpDatabase)).delete();
		try{
			copy(new File(mOpenedDatabasePath), new File(tmpDatabase));
		}
		catch(IOException e)
		{
			mToast.makeText(getActivity(), String.format("Failed to create temp database from %s!", mOpenedDatabasePath), mToast.LENGTH_LONG).show();
		}

		SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getActivity());
		boolean databaseInMemory = sharedPref.getBoolean(getString(R.string.pref_key_db_in_memory), Boolean.parseBoolean(getString(R.string.pref_default_db_in_memory)));		
		int status = RTABMapLib.openDatabase(tmpDatabase, databaseInMemory, optimize);
		setCamera(1);
		updateState(State.STATE_IDLE);
		
		if(status == -1)
		{
			new AlertDialog.Builder(getActivity())
			.setCancelable(false)
			.setTitle("Error")
			.setMessage("The map is loaded but optimization of the map's graph has "
					+ "failed, so the map cannot be shown. Change the Graph Optimizer approach used"
					+ " or enable/disable if the graph is optimized from graph "
					+ "end in \"Settings -> Mapping...\" and try opening again.")
			.setPositiveButton("Open Settings", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {
					Intent intent = new Intent(getActivity(), SettingsActivity.class);
					startActivity(intent);
				}
			})
			.setNegativeButton("Close", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {
				}
			})
			.show();
		}
	}
	
	public void copy(File src, File dst) throws IOException {
	    InputStream in = new FileInputStream(src);
	    OutputStream out = new FileOutputStream(dst);

	    // Transfer bytes from in to out
	    byte[] buf = new byte[1024];
	    int len;
	    while ((len = in.read(buf)) > 0) {
	        out.write(buf, 0, len);
	    }
	    in.close();
	    out.close();
	}
		
	private void shareToSketchfab()
	{
		Intent intent = new Intent(getActivity(), SketchfabActivity.class);
		
		intent.putExtra(RTABMAP_AUTH_TOKEN_KEY, mAuthToken);
		intent.putExtra(RTABMAP_EXPORTED_OBJ_KEY, mExportedOBJ);
		intent.putExtra(RTABMAP_WORKING_DIR_KEY, mWorkingDirectory);
		
		if(mOpenedDatabasePath.isEmpty())
		{
			intent.putExtra(RTABMAP_FILENAME_KEY, new SimpleDateFormat("yyMMdd-HHmmss").format(mDateOnPause));
		}
		else
		{
			File f = new File(mOpenedDatabasePath);
			String name = f.getName();
			intent.putExtra(RTABMAP_FILENAME_KEY, name.substring(0,name.lastIndexOf(".")));
		}
		
		startActivityForResult(intent, SKETCHFAB_ACTIVITY_CODE);
	}
}
