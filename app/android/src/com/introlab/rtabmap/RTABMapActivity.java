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
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
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
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
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
import android.text.Html;
import android.text.InputType;
import android.text.SpannableString;
import android.text.TextPaint;
import android.text.method.LinkMovementMethod;
import android.text.util.Linkify;
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
import android.widget.NumberPicker;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.google.atap.tangoservice.Tango;

// The main activity of the application. This activity shows debug information
// and a glSurfaceView that renders graphic content.
public class RTABMapActivity extends Activity implements OnClickListener {

	// Tag for debug logging.
	public static final String TAG = RTABMapActivity.class.getSimpleName();
	public static boolean DISABLE_LOG = true;

	// The minimum Tango Core version required from this application.
	private static final int  MIN_TANGO_CORE_VERSION = 9377;

	// The package name of Tang Core, used for checking minimum Tango Core version.
	private static final String TANGO_PACKAGE_NAME = "com.google.tango";

	public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
	public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";
	
	public static final String RTABMAP_TMP_DB = "rtabmap.tmp.db";
	public static final String RTABMAP_TMP_DIR = "tmp/";
	public static final String RTABMAP_TMP_FILENAME = "map";
	public static final String RTABMAP_SDCARD_PATH = "/sdcard/";
	public static final String RTABMAP_EXPORT_DIR = "Export/";

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
	ProgressDialog mExportProgressDialog;

	// Screen size for normalizing the touch input for orbiting the render camera.
	private Point mScreenSize = new Point();
	private long mOnPauseStamp = 0;
	private boolean mOnPause = false;
	private Date mDateOnPause = new Date();
	private boolean mBlockBack = true;

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
	private MenuItem mItemStatusVisibility;
	private MenuItem mItemDebugVisibility;

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
	private String mWorkingDirectoryHuman = "";

	private String mUpdateRate;
	private String mTimeThr;
	private String mMaxFeatures;
	private String mLoopThr;
	private String mMinInliers;
	private String mMaxOptimizationError;

	private int mTotalLoopClosures = 0;
	private boolean mMapIsEmpty = false;
	private boolean mExportedOBJ = false;
	int mMapNodes = 0;

	private Toast mToast = null;
	
	private AlertDialog mMemoryWarningDialog = null;
	
	private String[] mStatusTexts = new String[16];

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
		mButtonLighting.setChecked(false);
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

		mProgressDialog = new ProgressDialog(this);
		mProgressDialog.setCanceledOnTouchOutside(false);
		mRenderer.setProgressDialog(mProgressDialog);
		mRenderer.setToast(mToast);
		
		mExportProgressDialog = new ProgressDialog(this);
		mExportProgressDialog.setCanceledOnTouchOutside(false);
		mExportProgressDialog.setCancelable(false);
		mExportProgressDialog.setProgressStyle(ProgressDialog.STYLE_HORIZONTAL);
		mExportProgressDialog.setProgressNumberFormat(null);
		mExportProgressDialog.setProgressPercentFormat(null);
		mExportProgressDialog.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel", new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		    	RTABMapLib.cancelProcessing();
		    }
		});

		// Check if the Tango Core is out dated.
		if (!CheckTangoCoreVersion(MIN_TANGO_CORE_VERSION)) {
			mToast.makeText(this, "Tango Core out dated, please update in Play Store", mToast.LENGTH_LONG).show();
			finish();
			return;
		}   

		mOpenedDatabasePath = "";
		mWorkingDirectory = "";
		mWorkingDirectoryHuman = "";
		mTotalLoopClosures = 0;

		if(Environment.getExternalStorageState().compareTo(Environment.MEDIA_MOUNTED)==0)
		{
			File extStore = Environment.getExternalStorageDirectory();
			mWorkingDirectory = extStore.getAbsolutePath() + "/" + getString(R.string.app_name) + "/";
			extStore = new File(mWorkingDirectory);
			extStore.mkdirs();
			mWorkingDirectoryHuman = RTABMAP_SDCARD_PATH + getString(R.string.app_name) + "/";
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
        
       	DISABLE_LOG =  !( 0 != ( getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE ) );
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
	public void onBackPressed() {
	   
		if(mBlockBack)
		{
			mToast.makeText(this, "Press Back once more to exit", mToast.LENGTH_LONG).show();
			mBlockBack = false;
		}
		else
		{
			super.onBackPressed();
		}
	}
	
	@Override
	protected void onPause() {
		super.onPause();
		
		if(!DISABLE_LOG) Log.i(TAG, "onPause()");
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
			if(!DISABLE_LOG) Log.d(TAG, "update preferences...");
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
			
			if(!DISABLE_LOG) Log.d(TAG, "set mapping parameters");
			RTABMapLib.setNodesFiltering(sharedPref.getBoolean(getString(R.string.pref_key_nodes_filtering), Boolean.parseBoolean(getString(R.string.pref_default_nodes_filtering))));
			RTABMapLib.setAutoExposure(sharedPref.getBoolean(getString(R.string.pref_key_auto_exposure), Boolean.parseBoolean(getString(R.string.pref_default_auto_exposure))));
			RTABMapLib.setRawScanSaved(sharedPref.getBoolean(getString(R.string.pref_key_raw_scan_saved), Boolean.parseBoolean(getString(R.string.pref_default_raw_scan_saved))));
			RTABMapLib.setFullResolution(sharedPref.getBoolean(getString(R.string.pref_key_resolution), Boolean.parseBoolean(getString(R.string.pref_default_resolution))));
			RTABMapLib.setSmoothing(sharedPref.getBoolean(getString(R.string.pref_key_smoothing), Boolean.parseBoolean(getString(R.string.pref_default_smoothing))));
			RTABMapLib.setCameraColor(!sharedPref.getBoolean(getString(R.string.pref_key_fisheye), Boolean.parseBoolean(getString(R.string.pref_default_fisheye))));
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
	
			if(!DISABLE_LOG) Log.d(TAG, "set exporting parameters...");
			RTABMapLib.setCloudDensityLevel(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_density), getString(R.string.pref_default_density))));
			RTABMapLib.setMaxCloudDepth(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_depth), getString(R.string.pref_default_depth))));
			RTABMapLib.setMinCloudDepth(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_min_depth), getString(R.string.pref_default_min_depth))));
			RTABMapLib.setPointSize(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_point_size), getString(R.string.pref_default_point_size))));
			RTABMapLib.setMeshAngleTolerance(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_angle), getString(R.string.pref_default_angle))));
			RTABMapLib.setMeshTriangleSize(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_triangle), getString(R.string.pref_default_triangle))));
			
			if(!DISABLE_LOG) Log.d(TAG, "set rendering parameters...");
			RTABMapLib.setClusterRatio(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_cluster_ratio), getString(R.string.pref_default_cluster_ratio))));
			RTABMapLib.setMaxGainRadius(Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_gain_max_radius), getString(R.string.pref_default_gain_max_radius))));
			RTABMapLib.setRenderingTextureDecimation(Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_rendering_texture_decimation), getString(R.string.pref_default_rendering_texture_decimation))));
	
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

		if(!DISABLE_LOG) Log.i(TAG, String.format("onResume()"));

		if (Tango.hasPermission(this, Tango.PERMISSIONTYPE_MOTION_TRACKING)) {

			mGLView.onResume();

		} else {
			if(!DISABLE_LOG) Log.i(TAG, String.format("Asking for motion tracking permission"));
			startActivityForResult(
					Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
					Tango.TANGO_INTENT_ACTIVITYCODE);
		}
		
		TangoInitializationHelper.bindTangoService(getActivity(), mTangoServiceConnection);
	}
	
	private void setCamera(int type)
	{		
		// for convenience, for a refresh of the memory used
		mStatusTexts[1] = getString(R.string.memory)+String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024));
		mStatusTexts[2] = getString(R.string.free_memory)+String.valueOf(getFreeMemory());
		updateStatusTexts();
		
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
        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
        boolean fisheye = sharedPref.getBoolean(getString(R.string.pref_key_fisheye), Boolean.parseBoolean(getString(R.string.pref_default_fisheye)));
        Camera.getCameraInfo(fisheye?1:0, colorCameraInfo);
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
		if(!DISABLE_LOG) Log.i(TAG, "called onCreateOptionsMenu;");

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
		mItemStatusVisibility = menu.findItem(R.id.status);
		mItemDebugVisibility = menu.findItem(R.id.debug);
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
	
	private void updateStatusTexts()
	{
		if(mItemStatusVisibility != null && mItemDebugVisibility != null)
		{
			if(mItemStatusVisibility.isChecked() && mItemDebugVisibility.isChecked())
			{
				mRenderer.updateTexts(mStatusTexts);
			}
			else if(mItemStatusVisibility.isChecked())
			{
				mRenderer.updateTexts(Arrays.copyOfRange(mStatusTexts, 0, 3));
			}
			else if(mItemDebugVisibility.isChecked())
			{
				mRenderer.updateTexts(Arrays.copyOfRange(mStatusTexts, 4, mStatusTexts.length));
			}
			else
			{
				mRenderer.updateTexts(null);
			}
		}
	}

	private void updateStatsUI(
			int processMemoryUsed,
			int loopClosureId,
			int inliers,
			int matches,
			int rejected,
			float optimizationMaxError,
			String[] statusTexts)
	{
		mStatusTexts = statusTexts;
		updateStatusTexts();

		if(mButtonPause!=null)
		{
			if(!mButtonPause.isChecked())
			{	
				//check if we are low in memory
				long memoryUsed = processMemoryUsed;
				long memoryFree = getFreeMemory();
				
				if(memoryFree < 200)
				{
					mButtonPause.setChecked(true);
					pauseMapping();
					
					if(mMemoryWarningDialog!=null)
					{
						mMemoryWarningDialog.dismiss();
						mMemoryWarningDialog = null;
					}
					
					mMemoryWarningDialog = new AlertDialog.Builder(getActivity())
					.setTitle("Memory is full!")
					.setCancelable(false)
					.setMessage(String.format("Scanning has been paused because free memory is too "
							+ "low (%d MB). You should be able to save the database but some post-processing and exporting options may fail. "
							+ "\n\nNote that for large environments, you can save multiple databases and "
							+ "merge them with RTAB-Map Desktop version.", memoryUsed))
					.setPositiveButton("Ok", new DialogInterface.OnClickListener() {
						public void onClick(DialogInterface dialog, int which) {
							mMemoryWarningDialog = null;
						}
					})
					.setNeutralButton("Save", new DialogInterface.OnClickListener() {
						public void onClick(DialogInterface dialog, int which) {
							saveOnDevice();
							mMemoryWarningDialog = null;
						}
					})
					.create();
					mMemoryWarningDialog.show();
				}
				else if(mMemoryWarningDialog == null && memoryUsed*3 > memoryFree && (mItemDataRecorderMode == null || !mItemDataRecorderMode.isChecked()))
				{
					mMemoryWarningDialog = new AlertDialog.Builder(getActivity())
					.setTitle("Warning: Memory is almost full!")
					.setCancelable(false)
					.setMessage(String.format("Free memory (%d MB) should be at least 3 times the "
							+ "memory used (%d MB) so that some post-processing and exporting options "
							+ "have enough memory to work correctly. If you just want to save the database "
							+ "after scanning, you can continue until the next warning.\n\n"
							+ "Note that showing only point clouds reduces memory needed for rendering.", memoryFree, memoryUsed))
					.setPositiveButton("Pause", new DialogInterface.OnClickListener() {
						public void onClick(DialogInterface dialog, int which) {
							mButtonPause.setChecked(true);
							pauseMapping();
						}
					})
					.setNeutralButton("Continue", new DialogInterface.OnClickListener() {
						public void onClick(DialogInterface dialog, int which) {
						}
					})
					.create();
					mMemoryWarningDialog.show();
				}
			}
		}

		
		if(mButtonPause!=null && !mButtonPause.isChecked())
		{
			if(loopClosureId > 0)
			{
				mToast.setText(String.format("Loop closure detected! (%d/%d inliers)", inliers, matches));
				mToast.show();
			}
			else if(rejected > 0)
			{
				if(inliers >= Integer.parseInt(mMinInliers))
				{
					if(optimizationMaxError > 0.0f)
					{
						mToast.setText(String.format("Loop closure rejected, too high graph optimization error (%.3fm > %sm).", optimizationMaxError, mMaxOptimizationError));
					}
					else
					{
						mToast.setText(String.format("Loop closure rejected, graph optimization failed! You may try a different Graph Optimizer (see Mapping options)."));
					}
				}
				else
				{
					mToast.setText(String.format("Loop closure rejected, not enough inliers (%d/%d < %s).", inliers, matches, mMinInliers));
				}
				mToast.show();
			}
		}
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
			final int processMemoryUsed,
			final int databaseMemoryUsed,
			final int inliers,
			final int matches,
			final int featuresExtracted,
			final float hypothesis,
			final int nodesDrawn,
			final float fps,
			final int rejected,
			final float rehearsalValue,
			final float optimizationMaxError)
	{
		if(!DISABLE_LOG) Log.i(TAG, String.format("updateStatsCallback()"));

		final String[] statusTexts = new String[16];
		if(mButtonPause!=null && !mButtonPause.isChecked())
		{
			String updateValue = mUpdateRate.compareTo("0")==0?"Max":mUpdateRate;
			statusTexts[0] = getString(R.string.status)+(mItemDataRecorderMode!=null&&mItemDataRecorderMode.isChecked()?String.format("Recording (%s Hz)", updateValue):mItemLocalizationMode!=null && mItemLocalizationMode.isChecked()?String.format("Localization (%s Hz)", updateValue):String.format("Mapping (%s Hz)", updateValue));
		}
		else
		{
			statusTexts[0] = mStatusTexts[0];
		}
		
		// getNativeHeapAllocatedSize() is too slow, so we need to use the estimate.
		// Multiply by 3/2 to match getNativeHeapAllocatedSize()
		final int adjustedMemoryUsed = (processMemoryUsed*3)/2;
		
		if(mButtonPause!=null)
		{
			if(!mButtonPause.isChecked())
			{
				statusTexts[1] = getString(R.string.memory)+adjustedMemoryUsed; 
			}
			else if(mState == State.STATE_PROCESSING)
			{
				// This request is long to do, only do it when processing.
				statusTexts[1] = getString(R.string.memory)+String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024));
			}
			else
			{
				statusTexts[1] = mStatusTexts[1];
			}
		}
		else
		{
			statusTexts[1] = mStatusTexts[1];
		}
		statusTexts[2] = getString(R.string.free_memory)+getFreeMemory();
	
		
		if(loopClosureId > 0)
		{
			++mTotalLoopClosures;
		}
		
		mMapNodes = nodes;
		
		int index = 4;
		statusTexts[index++] = getString(R.string.nodes)+nodes+" (" + nodesDrawn + " shown)";
		statusTexts[index++] = getString(R.string.words)+words;
		statusTexts[index++] = getString(R.string.database_size)+databaseMemoryUsed;
		statusTexts[index++] = getString(R.string.points)+points;
		statusTexts[index++] = getString(R.string.polygons)+polygons;
		statusTexts[index++] = getString(R.string.update_time)+(int)(updateTime) + " / " + (mTimeThr.compareTo("0")==0?"No Limit":mTimeThr);
		statusTexts[index++] = getString(R.string.features)+featuresExtracted +" / " + (mMaxFeatures.compareTo("0")==0?"No Limit":mMaxFeatures.compareTo("-1")==0?"Disabled":mMaxFeatures);
		statusTexts[index++] = getString(R.string.rehearsal)+(int)(rehearsalValue*100.0f);
		statusTexts[index++] = getString(R.string.total_loop)+mTotalLoopClosures;
		statusTexts[index++] = getString(R.string.inliers)+inliers;
		statusTexts[index++] = getString(R.string.hypothesis)+(int)(hypothesis*100.0f) +" / " + (int)(Float.parseFloat(mLoopThr)*100.0f) + " (" + (loopClosureId>0?loopClosureId:highestHypId)+")";
		statusTexts[index++] = getString(R.string.fps)+(int)fps+" Hz";
	
		runOnUiThread(new Runnable() {
				public void run() {
					updateStatsUI(adjustedMemoryUsed, loopClosureId, inliers, matches, rejected, optimizationMaxError, statusTexts);
				} 
		});
	}

	private void rtabmapInitEventUI(
			int status, 
			String msg)
	{
		if(!DISABLE_LOG) Log.i(TAG, String.format("rtabmapInitEventsUI() status=%d msg=%s", status, msg));

		if(mButtonPause!=null)
		{
			if(mButtonPause.isChecked())
			{
				mStatusTexts[0] = getString(R.string.status)+(status == 1 && msg.isEmpty()?"Paused":msg);
			}
			else if(mItemLocalizationMode!=null && mItemDataRecorderMode!=null)
			{
				mStatusTexts[0] = getString(R.string.status)+(status == 1 && msg.isEmpty()?(mItemDataRecorderMode!=null&&mItemDataRecorderMode.isChecked()?"Recording":mItemLocalizationMode!=null&&mItemLocalizationMode.isChecked()?"Localization":"Mapping"):msg);
			}
			
			mStatusTexts[1] = getString(R.string.memory)+String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024));
			mStatusTexts[2] = getString(R.string.free_memory)+String.valueOf(getFreeMemory());
			updateStatusTexts();
		}
	}

	//called from jni
	public void rtabmapInitEventCallback(
			final int status, 
			final String msg)
	{
		if(!DISABLE_LOG) Log.i(TAG, String.format("rtabmapInitEventCallback()"));

		runOnUiThread(new Runnable() {
			public void run() {
				rtabmapInitEventUI(status, msg);
			} 
		});
	}
	
	private void updateProgressionUI(
			int count, 
			int max)
	{
		if(!DISABLE_LOG) Log.i(TAG, String.format("updateProgressionUI() count=%d max=%s", count, max));

		mExportProgressDialog.setMax(max);
		mExportProgressDialog.setProgress(count);
	}

	//called from jni
	public void updateProgressionCallback(
			final int count, 
			final int max)
	{
		if(!DISABLE_LOG) Log.i(TAG, String.format("updateProgressionCallback()"));

		runOnUiThread(new Runnable() {
			public void run() {
				updateProgressionUI(count, max);
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
		mExportProgressDialog.setTitle("Post-Processing");
		mExportProgressDialog.setMessage(String.format("Please wait while optimizing..."));
		mExportProgressDialog.setProgress(0);
		mExportProgressDialog.show();
		
		updateState(State.STATE_PROCESSING);
		Thread workingThread = new Thread(new Runnable() {
			public void run() {
				final int loopDetected = RTABMapLib.postProcessing(-1);
				runOnUiThread(new Runnable() {
					public void run() {
						if(mExportProgressDialog.isShowing())
						{
							mExportProgressDialog.dismiss();
							if(loopDetected >= 0)
							{
								mTotalLoopClosures+=loopDetected;
								mProgressDialog.setTitle("Post-Processing");
								mProgressDialog.setMessage(String.format("Optimization done! Increasing visual appeal..."));
								mProgressDialog.show();
							}
							else if(loopDetected < 0)
							{
								mToast.makeText(getActivity(), String.format("Optimization failed!"), mToast.LENGTH_LONG).show();
							}
						}
						else
						{
							mToast.makeText(getActivity(), String.format("Optimization canceled"), mToast.LENGTH_LONG).show();
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
		if(mState == State.STATE_VISUALIZING && state == State.STATE_IDLE && mMapNodes > 100)
		{
			mToast.makeText(getActivity(), String.format("Re-adding %d online clouds, this may take some time...", mMapNodes), mToast.LENGTH_LONG).show();
		}
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
			break;
		case STATE_VISUALIZING:
			mButtonLighting.setVisibility(View.VISIBLE);
			mButtonCloseVisualization.setVisibility(View.VISIBLE);
			mButtonSaveOnDevice.setVisibility(View.VISIBLE);
			mButtonShareOnSketchfab.setVisibility(View.VISIBLE);
			mItemSave.setEnabled(mButtonPause.isChecked());
			mItemExport.setEnabled(mButtonPause.isChecked() && !mItemDataRecorderMode.isChecked());
			mItemOpen.setEnabled(false);
			mItemPostProcessing.setEnabled(false);
			mItemSettings.setEnabled(true);
			mItemReset.setEnabled(true);
			mItemModes.setEnabled(true);
			mButtonPause.setVisibility(View.INVISIBLE);
			mItemDataRecorderMode.setEnabled(mButtonPause.isChecked());
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
			break;
		}
	}

	private void pauseMapping() {

		updateState(State.STATE_IDLE);

		if(mButtonPause.isChecked())
		{
			RTABMapLib.setPausedMapping(true);
			
			mStatusTexts[0] = getString(R.string.status)+"Paused";
			mStatusTexts[1] = getString(R.string.memory)+String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024));
			mStatusTexts[2] = getString(R.string.free_memory)+String.valueOf(getFreeMemory());
			updateStatusTexts();
			
			mMapIsEmpty = false;
			mDateOnPause = new Date();

			long memoryFree = getFreeMemory();
			if(!mOnPause && !mItemLocalizationMode.isChecked() && !mItemDataRecorderMode.isChecked() && memoryFree >= 100)
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
			if(mMemoryWarningDialog != null)
			{
				mMemoryWarningDialog.dismiss();
				mMemoryWarningDialog=null;
			}
			RTABMapLib.setPausedMapping(false);
			
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
		if(!DISABLE_LOG) Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
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
			mProgressDialog.setMessage(String.format("Adjusting Colors (Fast)..."));
			mProgressDialog.show();
			RTABMapLib.postProcessing(5);
		}
		else if (itemId == R.id.gain_compensation_full)
		{		
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Adjusting Colors (Full)..."));
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
		else if(itemId == R.id.status)
		{
			item.setChecked(!item.isChecked());
			updateStatusTexts();
		}
		else if(itemId == R.id.debug)
		{
			item.setChecked(!item.isChecked());
			updateStatusTexts();
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
									saveDatabase(fileName);
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
							saveDatabase(fileName);
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
			mTotalLoopClosures = 0;
			
			int index = 4;
			mMapNodes = 0;
			mStatusTexts[index++] = getString(R.string.nodes)+0;
			mStatusTexts[index++] = getString(R.string.words)+0;
			mStatusTexts[index++] = getString(R.string.database_size)+0;
			mStatusTexts[index++] = getString(R.string.points)+0;
			mStatusTexts[index++] = getString(R.string.polygons)+0;
			mStatusTexts[index++] = getString(R.string.update_time)+0;
			mStatusTexts[index++] = getString(R.string.features)+0;
			mStatusTexts[index++] = getString(R.string.rehearsal)+0;
			mStatusTexts[index++] = getString(R.string.total_loop)+0;
			mStatusTexts[index++] = getString(R.string.inliers)+0;
			mStatusTexts[index++] = getString(R.string.hypothesis)+0;
			mStatusTexts[index++] = getString(R.string.fps)+0;
			updateStatusTexts();

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
					mTotalLoopClosures = 0;
					int index = 4;
					mMapNodes = 0;
					mStatusTexts[index++] = getString(R.string.nodes)+0;
					mStatusTexts[index++] = getString(R.string.words)+0;
					mStatusTexts[index++] = getString(R.string.database_size)+0;
					mStatusTexts[index++] = getString(R.string.points)+0;
					mStatusTexts[index++] = getString(R.string.polygons)+0;
					mStatusTexts[index++] = getString(R.string.update_time)+0;
					mStatusTexts[index++] = getString(R.string.features)+0;
					mStatusTexts[index++] = getString(R.string.rehearsal)+0;
					mStatusTexts[index++] = getString(R.string.total_loop)+0;
					mStatusTexts[index++] = getString(R.string.inliers)+0;
					mStatusTexts[index++] = getString(R.string.hypothesis)+0;
					mStatusTexts[index++] = getString(R.string.fps)+0;
					updateStatusTexts();

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
				itemId == R.id.export_point_cloud_highrez ||
				itemId == R.id.export_mesh ||
				itemId == R.id.export_mesh_texture)
		{
			final boolean isOBJ = itemId == R.id.export_mesh_texture || itemId == R.id.export_optimized_mesh_texture;
			final boolean meshing = itemId != R.id.export_point_cloud && itemId != R.id.export_point_cloud_highrez;
			final boolean regenerateCloud = itemId == R.id.export_point_cloud_highrez;

			export(isOBJ, meshing, regenerateCloud, false, 0);
		}
		else if(itemId == R.id.export_optimized_mesh ||
				itemId == R.id.export_optimized_mesh_texture)
		{
			final boolean isOBJ = itemId == R.id.export_optimized_mesh_texture;
			
	        RelativeLayout linearLayout = new RelativeLayout(this);
	        final NumberPicker aNumberPicker = new NumberPicker(this);
	        aNumberPicker.setMaxValue(9);
	        aNumberPicker.setMinValue(0);
	        aNumberPicker.setWrapSelectorWheel(false);
	        aNumberPicker.setDescendantFocusability(NumberPicker.FOCUS_BLOCK_DESCENDANTS);
	        aNumberPicker.setFormatter(new NumberPicker.Formatter() {
	            @Override
	            public String format(int i) {
	            	if(i==0)
	            	{
	            		return "No Limit";
	            	}
	            	return String.format("%d00 000", i);
	            }
	        });
	        aNumberPicker.setValue(2);

	        // Fix to correctly show value on first render
	        try {
	        	Method method = aNumberPicker.getClass().getDeclaredMethod("changeValueByOne", boolean.class);
	        	method.setAccessible(true);
	        	method.invoke(aNumberPicker, true);
	        } catch (NoSuchMethodException e) {
	        	e.printStackTrace();
	        } catch (IllegalArgumentException e) {
	        	e.printStackTrace();
	        } catch (IllegalAccessException e) {
	        	e.printStackTrace();
	        } catch (InvocationTargetException e) {
	        	e.printStackTrace();
	        }


	        RelativeLayout.LayoutParams params = new RelativeLayout.LayoutParams(50, 50);
	        RelativeLayout.LayoutParams numPicerParams = new RelativeLayout.LayoutParams(RelativeLayout.LayoutParams.WRAP_CONTENT, RelativeLayout.LayoutParams.WRAP_CONTENT);
	        numPicerParams.addRule(RelativeLayout.CENTER_HORIZONTAL);

	        linearLayout.setLayoutParams(params);
	        linearLayout.addView(aNumberPicker,numPicerParams);

	        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(this);
	        alertDialogBuilder.setTitle("Maximum polygons");
	        alertDialogBuilder.setView(linearLayout);
	        alertDialogBuilder
	                .setCancelable(false)
	                .setPositiveButton("Ok",
	                        new DialogInterface.OnClickListener() {
	                            public void onClick(DialogInterface dialog,
	                                                int id) {
	                                export(isOBJ, true, false, true, aNumberPicker.getValue()*100000);
	                            }
	                        })
	                .setNegativeButton("Cancel",
	                        new DialogInterface.OnClickListener() {
	                            public void onClick(DialogInterface dialog,
	                                                int id) {
	                                dialog.cancel();
	                            }
	                        });
	        AlertDialog alertDialog = alertDialogBuilder.create();
	        alertDialog.show();
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
												
						// Adjust color now?
						new AlertDialog.Builder(getActivity())
						.setTitle("Opening database...")
						.setMessage("Do you want to adjust colors now?\nThis can be done later under Optimize menu.")
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
			mBlockBack = true;
		}
		else if(itemId == R.id.about)
		{
			AboutDialog about = new AboutDialog(this);
			about.setTitle("About RTAB-Map");
			about.show();
		}

		return true;
	}
	
	private void export(final boolean isOBJ, final boolean meshing, final boolean regenerateCloud, final boolean optimized, final int optimizedMaxPolygons)
	{
		final String extension = isOBJ? ".obj" : ".ply";
		
		// get Export settings
		SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
		final String cloudVoxelSizeStr = sharedPref.getString(getString(R.string.pref_key_cloud_voxel), getString(R.string.pref_default_cloud_voxel));
		final float cloudVoxelSize = Float.parseFloat(cloudVoxelSizeStr);
		final int textureSize = isOBJ?Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_texture_size), getString(R.string.pref_default_texture_size))):0;
		final int normalK = Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_normal_k), getString(R.string.pref_default_normal_k)));
		final float maxTextureDistance = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_max_texture_distance), getString(R.string.pref_default_max_texture_distance)));
		final int minTextureClusterSize = Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_min_texture_cluster_size), getString(R.string.pref_default_min_texture_cluster_size)));
		final float optimizedVoxelSize = cloudVoxelSize;
		final int optimizedDepth = Integer.parseInt(sharedPref.getString(getString(R.string.pref_key_opt_depth), getString(R.string.pref_default_opt_depth)));
		final float optimizedColorRadius = Float.parseFloat(sharedPref.getString(getString(R.string.pref_key_opt_color_radius), getString(R.string.pref_default_opt_color_radius)));
		final boolean optimizedCleanWhitePolygons = sharedPref.getBoolean(getString(R.string.pref_key_opt_clean_white), Boolean.parseBoolean(getString(R.string.pref_default_opt_clean_white)));
		final boolean optimizedColorWhitePolygons = false;//sharedPref.getBoolean("pref_key_opt_color_white", false); // not used
		final boolean blockRendering = sharedPref.getBoolean(getString(R.string.pref_key_block_render), Boolean.parseBoolean(getString(R.string.pref_default_block_render)));

		
		mExportProgressDialog.setTitle("Exporting");
		mExportProgressDialog.setMessage(String.format("Please wait while preparing data to export..."));
		mExportProgressDialog.setProgress(0);
		
		final State previousState = mState;
		
		mExportProgressDialog.show();
		updateState(State.STATE_PROCESSING);
		final String tmpPath = mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + extension;
	
		File tmpDir = new File(mWorkingDirectory + RTABMAP_TMP_DIR);
		tmpDir.mkdirs();
		File exportDir = new File(mWorkingDirectory + RTABMAP_EXPORT_DIR);
		exportDir.mkdirs();
		
		Thread exportThread = new Thread(new Runnable() {
			public void run() {

				final long startTime = System.currentTimeMillis()/1000;

				final boolean success = RTABMapLib.exportMesh(
						tmpPath,
						cloudVoxelSize,
						regenerateCloud,
						meshing,
						textureSize,
						normalK,
						optimized,
						optimizedVoxelSize,
						optimizedDepth,
						optimizedMaxPolygons,
						optimizedColorRadius,
						optimizedCleanWhitePolygons,
						optimizedColorWhitePolygons,
						maxTextureDistance,
						minTextureClusterSize,
						blockRendering);
				runOnUiThread(new Runnable() {
					public void run() {
						if(mExportProgressDialog.isShowing())
						{
							if(success)
							{
								if(!meshing && cloudVoxelSize>0.0f)
								{
									mToast.makeText(getActivity(), String.format("Cloud assembled and voxelized at %s m.", cloudVoxelSizeStr), mToast.LENGTH_LONG).show();
								}
								
								final long endTime = System.currentTimeMillis()/1000;

								// Visualize the result?
								AlertDialog d = new AlertDialog.Builder(getActivity())
								.setCancelable(false)
								.setTitle("Export Successful! (" + (endTime-startTime) + " sec)")
								.setMessage(Html.fromHtml("Do you want visualize the result before saving to file or sharing to <a href=\"https://sketchfab.com/about\">Sketchfab</a>?"))
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
										
										AlertDialog d2 = new AlertDialog.Builder(getActivity())
										.setCancelable(false)
										.setTitle("Save to...")
										.setMessage(Html.fromHtml("Do you want to share to <a href=\"https://sketchfab.com/about\">Sketchfab</a> or save it on device?"))
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
										.setNeutralButton("Cancel", new DialogInterface.OnClickListener() {
											public void onClick(DialogInterface dialog, int which) {
											}
										})
										.create();
										d2.show();
										// Make the textview clickable. Must be called after show()
									    ((TextView)d2.findViewById(android.R.id.message)).setMovementMethod(LinkMovementMethod.getInstance());
									}
								})
								.create();
								d.show();
								// Make the textview clickable. Must be called after show()
							    ((TextView)d.findViewById(android.R.id.message)).setMovementMethod(LinkMovementMethod.getInstance());
							}
							else
							{
								updateState(State.STATE_IDLE);
								mToast.makeText(getActivity(), String.format("Exporting map failed!"), mToast.LENGTH_LONG).show();
							}
							mExportProgressDialog.dismiss();
						}
						else
						{
							mToast.makeText(getActivity(), String.format("Export canceled"), mToast.LENGTH_LONG).show();
							updateState(previousState);
						}
					}
				});
			} 
		});
		exportThread.start();
	}
	
	private void saveDatabase(String fileName)
	{
		final String newDatabasePath = mWorkingDirectory + fileName + ".db";
		final String newDatabasePathHuman = mWorkingDirectoryHuman + fileName + ".db";
		mProgressDialog.setTitle("Saving");
		if(mOpenedDatabasePath.equals(newDatabasePath))
		{
			mProgressDialog.setMessage(String.format("Please wait while updating \"%s\"...", newDatabasePathHuman));
		}
		else
		{
			mProgressDialog.setMessage(String.format("Please wait while saving \"%s\"...", newDatabasePathHuman));
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
							mToast.makeText(getActivity(), String.format("Database \"%s\" updated.", newDatabasePathHuman), mToast.LENGTH_LONG).show();
						}
						else
						{
							mToast.makeText(getActivity(), String.format("Database saved to \"%s\".", newDatabasePathHuman), mToast.LENGTH_LONG).show();

							Intent intent = new Intent(getActivity(), RTABMapActivity.class);
							// use System.currentTimeMillis() to have a unique ID for the pending intent
							PendingIntent pIntent = PendingIntent.getActivity(getActivity(), (int) System.currentTimeMillis(), intent, 0);

							// build notification
							// the addAction re-use the same intent to keep the example short
							Notification n  = new Notification.Builder(getActivity())
							.setContentTitle(getString(R.string.app_name))
							.setContentText(newDatabasePathHuman + " saved!")
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
					File newFile = new File(mWorkingDirectory + RTABMAP_EXPORT_DIR + fileName + (mExportedOBJ?".zip":".ply"));
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
		String pathHuman;
		boolean success = true;
		if(mExportedOBJ)
		{	
			final String zipOutput = mWorkingDirectory+RTABMAP_EXPORT_DIR+fileName+".zip";
			pathHuman = mWorkingDirectoryHuman + RTABMAP_EXPORT_DIR + fileName + ".zip";

			String[] filesToZip = new String[3];
			filesToZip[0] = mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".obj";
			filesToZip[1] = mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".mtl";
			filesToZip[2] = mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".jpg";

			File toZIPFile = new File(zipOutput);
			toZIPFile.delete();			
			
			try
			{
				Util.zip(filesToZip, zipOutput);
				mToast.makeText(getActivity(), String.format("Mesh \"%s\" successfully exported!", pathHuman), mToast.LENGTH_LONG).show();
			}
			catch(IOException e)
			{
				mToast.makeText(getActivity(), String.format("Exporting mesh \"%s\" failed! Error=%s", pathHuman, e.getMessage()), mToast.LENGTH_LONG).show();
				success = false;
			}
		}
		else
		{
			final String path = mWorkingDirectory + RTABMAP_EXPORT_DIR+ fileName + ".ply";
			pathHuman = mWorkingDirectoryHuman + RTABMAP_EXPORT_DIR + fileName + ".ply";
			
			File toPLYFile = new File(path);
			toPLYFile.delete();
			File fromPLYFile = new File(mWorkingDirectory + RTABMAP_TMP_DIR + RTABMAP_TMP_FILENAME + ".ply");
			
			try
			{
				copy(fromPLYFile,toPLYFile);
				mToast.makeText(getActivity(), String.format("Mesh/point cloud \"%s\" successfully exported!", pathHuman), mToast.LENGTH_LONG).show();
			}
			catch(Exception e)
			{
				mToast.makeText(getActivity(), String.format("Exporting mesh/point cloud \"%s\" failed! Error=%s", pathHuman, e.getMessage()), mToast.LENGTH_LONG).show();
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
			.setContentText(pathHuman + " exported!")
			.setSmallIcon(R.drawable.ic_launcher)
			.setContentIntent(pIntent)
			.setAutoCancel(true).build();


			NotificationManager notificationManager = 
					(NotificationManager) getSystemService(NOTIFICATION_SERVICE);

			notificationManager.notify(0, n); 
		}
	}
	
	private void openDatabase(final String fileName, final boolean optimize)
	{
		mOpenedDatabasePath = mWorkingDirectory + fileName;
		
		SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getActivity());
		final boolean databaseInMemory = sharedPref.getBoolean(getString(R.string.pref_key_db_in_memory), Boolean.parseBoolean(getString(R.string.pref_default_db_in_memory)));		
		
		
		mProgressDialog.setTitle("Loading");
		mProgressDialog.setMessage(String.format("Opening database \"%s\"...", fileName));
		mProgressDialog.show();
		updateState(State.STATE_PROCESSING);
		
		Thread openThread = new Thread(new Runnable() {
			public void run() {

				final String tmpDatabase = mWorkingDirectory+RTABMAP_TMP_DB;
				(new File(tmpDatabase)).delete();
				try{
					copy(new File(mOpenedDatabasePath), new File(tmpDatabase));
				}
				catch(IOException e)
				{
					mToast.makeText(getActivity(), String.format("Failed to create temp database from %s!", mOpenedDatabasePath), mToast.LENGTH_LONG).show();
					updateState(State.STATE_IDLE);
					mProgressDialog.dismiss();
					return;
				}
				
				final int status = RTABMapLib.openDatabase(tmpDatabase, databaseInMemory, optimize);
				
				runOnUiThread(new Runnable() {
					public void run() {
						setCamera(1);
						updateState(State.STATE_IDLE);
						if(status == -1)
						{
							mProgressDialog.dismiss();
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
									mBlockBack = true;
								}
							})
							.setNegativeButton("Close", new DialogInterface.OnClickListener() {
								public void onClick(DialogInterface dialog, int which) {
								}
							})
							.show();
						}
						else
						{
							// creating meshes...
							if(!mItemTrajectoryMode.isChecked())
							{
								mProgressDialog.setTitle("Loading");
								mProgressDialog.setMessage(String.format("Database \"%s\" loaded. Please wait while rendering point clouds and meshes...", fileName));
							}
						}
					}
				});
			} 
		});
		openThread.start();	
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
		mBlockBack = true;
	}
}
