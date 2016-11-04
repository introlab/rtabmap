package com.introlab.rtabmap;

import java.io.File;
import java.io.FilenameFilter;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.ProgressDialog;
import android.content.ComponentName;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.content.pm.PackageManager.NameNotFoundException;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Debug;
import android.os.IBinder;
import android.text.Editable;
import android.text.InputType;
import android.util.Log;
import android.view.Display;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MenuInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;

// The main activity of the application. This activity shows debug information
// and a glSurfaceView that renders graphic content.
public class RTABMapActivity extends Activity implements OnClickListener {

  // Tag for debug logging.
  private static final String TAG = RTABMapActivity.class.getSimpleName();

  // The minimum Tango Core version required from this application.
  private static final int  MIN_TANGO_CORE_VERSION = 9377;

  // The package name of Tang Core, used for checking minimum Tango Core version.
  private static final String TANGO_PACKAGE_NAME = "com.google.tango";
  
  public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
  public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";

  // GLSurfaceView and renderer, all of the graphic content is rendered
  // through OpenGL ES 2.0 in native code.
  private Renderer mRenderer;
  private GLSurfaceView mGLView;
  
  ProgressDialog mProgressDialog;

  // Screen size for normalizing the touch input for orbiting the render camera.
  private Point mScreenSize = new Point();
  private boolean mPauseFirstTime = true;
  private boolean mOnPause = false;
  
  private MenuItem mItemPause;
  private MenuItem mItemSave;
  private MenuItem mItemOpen;
  private MenuItem mItemPostProcessing;
  private MenuItem mItemExport;
  private MenuItem mItemLocalizationMode;
  private MenuItem mItemTrajectoryMode;
  private MenuItem mItemRenderingPointCloud;
  private MenuItem mItemRenderingMesh;
  private MenuItem mItemRenderingTextureMesh;
  private MenuItem mItemDataRecorderMode;
  
  
  private String mOpenedDatabasePath = "";
  private String mTempDatabasePath = "";
  private String mNewDatabasePath = "";
  private String mWorkingDirectory = "";
  
  private int mMaxDepthIndex = 5;
  private int mMeshAngleToleranceIndex = 2;
  private int mMeshTriangleSizeIndex = 0;
  
  private int mParamUpdateRateHzIndex = 1;
  private int mParamTimeThrMsIndex = 4;
  private int mParamMaxFeaturesIndex = 2;
  private int mParamLoopThrMsIndex = 1;
  private int mParamOptimizeErrorIndex = 3;
  
  final String[] mUpdateRateValues = {"0.5", "1", "2", "Max"};
  final String[] mTimeThrValues = {"400", "500", "600", "700", "800", "900", "1000", "1100", "1200", "1300", "1400", "1500", "No Limit"};
  final String[] mMaxFeaturesValues = {"Disabled", "100", "200", "300", "400", "500", "600", "700", "800", "900", "1000", "No Limit"};
  final String[] mLoopThrValues = {"Disabled", "0.11", "0.20", "0.30", "0.40", "0.50", "0.60", "0.70", "0.80", "0.90"};
  final String[] mOptimizeErrorValues = {"Disabled", "0.01", "0.025", "0.05", "0.1", "0.2", "0.35", "0.5", "1"};
  
  private LinearLayout mLayoutDebug;
  
  private int mTotalLoopClosures = 0;
  private boolean mMapIsEmpty = false;
  
  private Toast mToast = null;
  
  //Tango Service connection.
  ServiceConnection mTangoServiceConnection = new ServiceConnection() {
     public void onServiceConnected(ComponentName name, IBinder service) {
       if(!RTABMapLib.onTangoServiceConnected(service))
       {
    	   mToast.makeText(getApplicationContext(), 
    				String.format("Failed to intialize Tango!"), mToast.LENGTH_SHORT).show();
       }
     }

     public void onServiceDisconnected(ComponentName name) {
       // Handle this if you need to gracefully shutdown/retry
       // in the event that Tango itself crashes/gets upgraded while running.
       mToast.makeText(getApplicationContext(), 
 				String.format("Tango disconnected!"), mToast.LENGTH_SHORT).show();
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

    // Buttons for selecting camera view and Set up button click listeners.
    findViewById(R.id.first_person_button).setOnClickListener(this);
    findViewById(R.id.third_person_button).setOnClickListener(this);
    findViewById(R.id.top_down_button).setOnClickListener(this);
    
    mToast = Toast.makeText(getApplicationContext(), "", Toast.LENGTH_SHORT);

    // OpenGL view where all of the graphics are drawn.
    mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
    
    // Configure OpenGL renderer
    mGLView.setEGLContextClientVersion(2);

    // Configure the OpenGL renderer.
    mRenderer = new Renderer();
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
    mTempDatabasePath = "";
    mNewDatabasePath = "";
    mWorkingDirectory = "";
    mTotalLoopClosures = 0;
    
    if(Environment.getExternalStorageState().compareTo(Environment.MEDIA_MOUNTED)==0)
    {
    	File extStore = Environment.getExternalStorageDirectory();
    	mWorkingDirectory = extStore.getAbsolutePath() + "/" + getString(R.string.app_name) + "/";
    	extStore = new File(mWorkingDirectory);
    	extStore.mkdirs();
    	mTempDatabasePath = mWorkingDirectory + "rtabmap.tmp.db";
    	extStore = new File(mTempDatabasePath);
    	
    	if(extStore.exists())
    	{
    		extStore.delete();
    	}   	
    }
    else
    {
    	// show warning that data cannot be saved!
		mToast.makeText(getApplicationContext(), 
				String.format("Failed to get external storage path (SD-CARD, state=%s). Saving disabled.", 
						Environment.getExternalStorageState()), mToast.LENGTH_LONG).show();
    }
    
    RTABMapLib.onCreate(this);
    RTABMapLib.openDatabase(mTempDatabasePath);
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
  }

  @Override
  protected void onResume() {
    super.onResume();

    mProgressDialog.setTitle("");
	mProgressDialog.setMessage(String.format("Hold Tight! Initializing Tango Service..."));
	mProgressDialog.show();
    
	if(mOnPause)
	{
		mToast.makeText(this, "Mapping is paused!", mToast.LENGTH_LONG).show();
	}
	else
	{
		mToast.makeText(this, "Tip: If the camera is still drifting just after the mapping has started, do \"Reset\".", mToast.LENGTH_LONG).show();
	}
	
	mOnPause = false;
    
    TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    
    Log.i(TAG, String.format("onResume()"));
    
    if (Tango.hasPermission(this, Tango.PERMISSIONTYPE_MOTION_TRACKING)) {

    	mGLView.onResume();

    } else {
    	Log.i(TAG, String.format("Asking for motion tracking permission"));
        startActivityForResult(
                Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                Tango.TANGO_INTENT_ACTIVITYCODE);
    }
  }

  @Override
  protected void onPause() {
    super.onPause();
    
    // This deletes OpenGL context!
    mGLView.onPause();
    
    mOnPause = true;
    
    RTABMapLib.onPause();
    
    unbindService(mTangoServiceConnection);
    
    if(!mItemPause.isChecked())
    {
    	onOptionsItemSelected(mItemPause);
    }
  }

  @Override
  public void onClick(View v) {
    // Handle button clicks.
    switch (v.getId()) {
    case R.id.first_person_button:
      RTABMapLib.setCamera(0);
      break;
    case R.id.third_person_button:
      RTABMapLib.setCamera(1);
      break;
    case R.id.top_down_button:
      RTABMapLib.setCamera(2);
      break;
    default:
      return;
    }
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
	  
	  mItemPause = menu.findItem(R.id.pause);
	  mItemSave = menu.findItem(R.id.save);
	  mItemOpen = menu.findItem(R.id.open);
	  mItemPostProcessing = menu.findItem(R.id.post_processing);
	  mItemExport = menu.findItem(R.id.export);
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
	  
	  return true;
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
		  int featuresExtracted,
		  float hypothesis,
		  int nodesDrawn,
		  float fps,
		  int rejected)
  {
	  if(mItemPause!=null)
	  {
		  ((TextView)findViewById(R.id.status)).setText(mItemPause.isChecked()?"Paused":mItemLocalizationMode.isChecked()?String.format("Localization (%s Hz)", mUpdateRateValues[mParamUpdateRateHzIndex]):String.format("Mapping (%s Hz)", mUpdateRateValues[mParamUpdateRateHzIndex]));
	  }
	  
	  ((TextView)findViewById(R.id.points)).setText(String.valueOf(points));
	  ((TextView)findViewById(R.id.polygons)).setText(String.valueOf(polygons));
	  ((TextView)findViewById(R.id.nodes)).setText(String.format("%d (%d shown)", nodes, nodesDrawn));
	  ((TextView)findViewById(R.id.words)).setText(String.valueOf(words));
	  ((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
	  ((TextView)findViewById(R.id.db_size)).setText(String.valueOf(databaseMemoryUsed));
	  ((TextView)findViewById(R.id.inliers)).setText(String.valueOf(inliers));
	  ((TextView)findViewById(R.id.features)).setText(String.format("%d / %s", featuresExtracted, mMaxFeaturesValues[mParamMaxFeaturesIndex]));
	  ((TextView)findViewById(R.id.update_time)).setText(String.format("%.3f / %s", updateTime, mTimeThrValues[mParamTimeThrMsIndex]));
	  ((TextView)findViewById(R.id.hypothesis)).setText(String.format("%.3f / %s (%d)", hypothesis, mLoopThrValues[mParamLoopThrMsIndex], loopClosureId>0?loopClosureId:highestHypId));
	  ((TextView)findViewById(R.id.fps)).setText(String.format("%.3f Hz", fps));
	  if(mItemPause!=null && !mItemPause.isChecked())
	  {
		  if(loopClosureId > 0)
		  {
			  ++mTotalLoopClosures;
	
			  mToast.setText("Loop closure detected!");
			  mToast.show();
		  }
		  else if(rejected > 0 && inliers >= 15)
		  {
			  mToast.setText("Loop closure rejected after graph optimization.");
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
		  final int features,
		  final float hypothesis,
		  final int nodesDrawn,
		  final float fps,
		  final int rejected)
  {
	  Log.i(TAG, String.format("updateStatsCallback()"));

	  runOnUiThread(new Runnable() {
		    public void run() {
		    	updateStatsUI(nodes, words, points, polygons, updateTime, loopClosureId, highestHypId, databaseMemoryUsed, inliers, features, hypothesis, nodesDrawn, fps, rejected);
		    } 
		});
  }
  
  private void rtabmapInitEventUI(
		  int status, 
		  String msg)
  {
	  Log.i(TAG, String.format("rtabmapInitEventsUI() status=%d msg=%s", status, msg));
	  
	  ((TextView)findViewById(R.id.status)).setText(
			  status == 1 && msg.isEmpty()?mItemPause!=null&&mItemPause.isChecked()?"Paused":mItemLocalizationMode!=null&&mItemLocalizationMode.isChecked()?"Localization":"Mapping":msg);
	  
	  /*0=kInitializing,
		1=kInitialized,
		2=kClosing,
		3=kClosed,
		4=kInfo,
		5=kError*/
	  
		if(status == 3)
		{
			msg = "";
			if(!mNewDatabasePath.isEmpty())
			{
				boolean removed = true;
				File outputFile = new File(mNewDatabasePath);
				if(outputFile.exists())
				{
					removed = outputFile.delete();
				}
				if(removed)
				{
					File tempFile = new File(mTempDatabasePath);
					if(tempFile.renameTo(outputFile))
					{
						msg = String.format("Database saved to \"%s\". Tip: You can open a saved database with \"Open\".", mNewDatabasePath);
						
						Intent intent = new Intent(this, RTABMapActivity.class);
						// use System.currentTimeMillis() to have a unique ID for the pending intent
						PendingIntent pIntent = PendingIntent.getActivity(this, (int) System.currentTimeMillis(), intent, 0);

						// build notification
						// the addAction re-use the same intent to keep the example short
						Notification n  = new Notification.Builder(this)
						        .setContentTitle(getString(R.string.app_name))
						        .setContentText(mNewDatabasePath + " saved!")
						        .setSmallIcon(R.drawable.ic_launcher)
						        .setContentIntent(pIntent)
						        .setAutoCancel(true).build();
						    
						  
						NotificationManager notificationManager = 
						  (NotificationManager) getSystemService(NOTIFICATION_SERVICE);

						notificationManager.notify(0, n); 
					}
					else
					{
						msg = String.format("Failed to rename temporary database from \"%s\" to \"%s\".",
								mTempDatabasePath, mNewDatabasePath);
					}
				}
				else
				{
					msg = String.format("Failed to overwrite the database \"%s\". The temporary database is still correctly saved at \"%s\".",
							mNewDatabasePath, mTempDatabasePath);
				}
			}
			else if(!mOpenedDatabasePath.isEmpty())
			{
				msg = String.format("Database \"%s\" updated.", mOpenedDatabasePath);
			}

			if(!msg.isEmpty())
			{
				mToast.makeText(this, msg, mToast.LENGTH_LONG).show();
			}
			
			mOpenedDatabasePath = "";
			mNewDatabasePath = "";
			
			//restart a new scan by default
			RTABMapLib.openDatabase(mTempDatabasePath);
			
			((TextView)findViewById(R.id.points)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.polygons)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.nodes)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.words)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
			((TextView)findViewById(R.id.db_size)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.inliers)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.features)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.update_time)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.hypothesis)).setText(String.valueOf(0));
			((TextView)findViewById(R.id.fps)).setText(String.valueOf(0));
			mTotalLoopClosures = 0;
			((TextView)findViewById(R.id.total_loop)).setText(String.valueOf(mTotalLoopClosures));
			
			if(mItemSave!=null)
			{
				if(mItemPause.isChecked())
				{
					mItemPause.setChecked(false);
					mItemOpen.setEnabled(false);
					mItemPostProcessing.setEnabled(false);
					mItemSave.setEnabled(false);
					mItemExport.setEnabled(false);
					mItemDataRecorderMode.setEnabled(false);
					RTABMapLib.setPausedMapping(false); // resume mapping
				}
			}
			mProgressDialog.dismiss();
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
	  if(mItemPause != null && !mItemPause.isChecked())
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
	                return filename.endsWith(".db");
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
  
  public boolean onOptionsItemSelected(MenuItem item) {
	  Log.i(TAG, "called onOptionsItemSelected; selected item: " + item);
	  int itemId = item.getItemId();
      if (itemId == R.id.pause)
      {
    	  item.setChecked(!item.isChecked());
    	  mItemSave.setEnabled(item.isChecked());
		  mItemExport.setEnabled(item.isChecked() && !mItemDataRecorderMode.isChecked());
    	  mItemOpen.setEnabled(item.isChecked() && !mItemDataRecorderMode.isChecked());
    	  mItemPostProcessing.setEnabled(item.isChecked() && !mItemDataRecorderMode.isChecked());
    	  mItemDataRecorderMode.setEnabled(item.isChecked());
    	 // mItemSave.setEnabled(item.isChecked() && !mWorkingDirectory.isEmpty());
    	  if(item.isChecked())
    	  {
    		  RTABMapLib.setPausedMapping(true);
    		  ((TextView)findViewById(R.id.status)).setText("Paused");
    		  if(mPauseFirstTime && !mOnPause)
    		  {
    			  mPauseFirstTime = false;
    			  mToast.makeText(getActivity(), String.format("Tip: Try \"Post-Processing...\" to optimize even more the map!"), mToast.LENGTH_LONG).show();
    		  }
    		  mMapIsEmpty = false;
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
      else if (itemId == R.id.post_processing_standard)
      {
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Please wait while optimizing..."));
			mProgressDialog.show();
			
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
					    	}
				    	});
				    } 
			});
			workingThread.start();
      }
      else if (itemId == R.id.detect_more_loop_closures)
      {
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Please wait while detecting more loop closures..."));
			mProgressDialog.show();
			
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
				    		}
				    	});
				    } 
			});
			workingThread.start();
      }
      else if (itemId == R.id.icp_refining)
      {
			mProgressDialog.setTitle("Post-Processing");
			mProgressDialog.setMessage(String.format("Please wait while refining links..."));
			mProgressDialog.show();
			
			Thread workingThread = new Thread(new Runnable() {
				    public void run() {
				    	final int linksRefined = RTABMapLib.postProcessing(3);
				    	runOnUiThread(new Runnable() {
				    		public void run() {
								mProgressDialog.dismiss();
								if(linksRefined >= 0)
								{
									mToast.makeText(getActivity(), String.format("Refining done! %d link(s) refined.", linksRefined), mToast.LENGTH_SHORT).show();
								}
								else if(linksRefined < 0)
								{
									mToast.makeText(getActivity(), String.format("Refining failed!"), mToast.LENGTH_SHORT).show();
								}
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
      }
      else if(itemId == R.id.graph_optimization)
      {
    	  item.setChecked(!item.isChecked());
    	  RTABMapLib.setGraphOptimization(item.isChecked());
      }
      else if(itemId == R.id.nodes_filtering)
      {
    	  item.setChecked(!item.isChecked());
    	  RTABMapLib.setNodesFiltering(item.isChecked());
      }
      else if(itemId == R.id.drift_correction)
      {
    	  item.setChecked(!item.isChecked());
    	  RTABMapLib.setDriftCorrection(item.isChecked());
    	  if(item.isChecked())
    	  {
    		  mToast.makeText(getActivity(), String.format("Tip: With drift correction is enabled, move slowly to get better results."), mToast.LENGTH_LONG).show();
    	  }
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
      else if(itemId == R.id.auto_exposure)
      {
    	  item.setChecked(!item.isChecked());
    	  RTABMapLib.setAutoExposure(item.isChecked());
    	  
    	  // restart Tango service
    	  onPause();
    	  onResume();    	  
      }
      else if(itemId == R.id.resolution)
      {
    	  item.setChecked(!item.isChecked());
    	  RTABMapLib.setFullResolution(item.isChecked());
      }
      else if(itemId == R.id.max_depth)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Max Depth (m)");
		  final String[] values = {"1", "2", "3", "4", "5", "No Limit"};
		  builder.setSingleChoiceItems(values, mMaxDepthIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < 6)
		        {
			        mMaxDepthIndex = which;
			        RTABMapLib.setMaxCloudDepth(which < 5?Float.parseFloat(values[which]):0);
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.mesh_angle_tolerance)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Mesh Angle Tolerance (deg)");
		  final String[] values = {"5", "10", "15", "20", "25", "30"};
		  builder.setSingleChoiceItems(values, mMeshAngleToleranceIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < 6)
		        {
		        	mMeshAngleToleranceIndex = which;
			        RTABMapLib.setMeshAngleTolerance(Float.parseFloat(values[which]));
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.mesh_triangle_size)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Mesh Triangle Size (pixels)");
		  final String[] values = {"2", "3", "4", "5", "6"};
		  builder.setSingleChoiceItems(values, mMeshTriangleSizeIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < 5)
		        {
		        	mMeshTriangleSizeIndex = which;
			        RTABMapLib.setMeshTriangleSize(Integer.parseInt(values[which]));
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.update_rate)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Update Rate (Hz)");
		  builder.setSingleChoiceItems(mUpdateRateValues, mParamUpdateRateHzIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < mUpdateRateValues.length)
		        {
		        	mParamUpdateRateHzIndex = which;
			        if(RTABMapLib.setMappingParameter("Rtabmap/DetectionRate", mUpdateRateValues[which]) != 0)
			        {
			        	mToast.makeText(getActivity(), "Failed to set parameter \"Rtabmap/DetectionRate\"!", mToast.LENGTH_LONG).show();
			        }
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.time_threshold)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Time Threshold (ms)");
		  builder.setSingleChoiceItems(mTimeThrValues, mParamTimeThrMsIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < mTimeThrValues.length)
		        {
		        	mParamTimeThrMsIndex = which;
			        if(RTABMapLib.setMappingParameter("Rtabmap/TimeThr", which==mTimeThrValues.length-1?"0":mTimeThrValues[which]) != 0)
			        {
			        	mToast.makeText(getActivity(), "Failed to set parameter \"Rtabmap/TimeThr\"!", mToast.LENGTH_LONG).show();
			        }
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.features)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Max Features");
		  builder.setSingleChoiceItems(mMaxFeaturesValues, mParamMaxFeaturesIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < mMaxFeaturesValues.length)
		        {
		        	mParamMaxFeaturesIndex = which;
			        if(RTABMapLib.setMappingParameter("Kp/MaxFeatures", which==0?"-1":which==mMaxFeaturesValues.length-1?"0":mMaxFeaturesValues[which]) != 0)
			        {
			        	mToast.makeText(getActivity(),"Failed to set parameter \"Kp/MaxFeatures\"!", mToast.LENGTH_LONG).show();
			        }
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.loop_threshold)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Loop Closure Threshold");
		  builder.setSingleChoiceItems(mLoopThrValues, mParamLoopThrMsIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < mLoopThrValues.length)
		        {
		        	mParamLoopThrMsIndex = which;
			        if(RTABMapLib.setMappingParameter("Rtabmap/LoopThr", which==0?"1":mLoopThrValues[which]) != 0)
			        {
			        	mToast.makeText(getActivity(), "Failed to set parameter \"Rtabmap/LoopThr\"!", mToast.LENGTH_LONG).show();
			        }
		        }
		    }
		  });
		  builder.show();
      }
      else if(itemId == R.id.optimize_error)
      {
    	  // get double
		  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle("Max Optimization Error (m)");
		  builder.setSingleChoiceItems(mOptimizeErrorValues, mParamOptimizeErrorIndex, new DialogInterface.OnClickListener() {
		    @Override
		    public void onClick(DialogInterface dialog, int which) {
		        dialog.dismiss();
		        if(which >=0 && which < mOptimizeErrorValues.length)
		        {
		        	mParamOptimizeErrorIndex = which;
			        if(RTABMapLib.setMappingParameter("RGBD/OptimizeMaxError", which==0?"0":mOptimizeErrorValues[which]) != 0)
			        {
			        	mToast.makeText(getActivity(), "Failed to set parameter \"RGBD/OptimizeMaxError\"!", mToast.LENGTH_LONG).show();
			        }
		        }
		    }
		  });
		  builder.show();
      }
      else if (itemId == R.id.save)
      {
    	  if(mOpenedDatabasePath.isEmpty())
    	  {
			  AlertDialog.Builder builder = new AlertDialog.Builder(this);
			  builder.setTitle("RTAB-Map Database Name (*.db):");
			  final EditText input = new EditText(this);
			  input.setInputType(InputType.TYPE_CLASS_TEXT); 
			  String timeStamp = new SimpleDateFormat("yyMMdd-hhmmss").format(new Date());
			  input.setText(timeStamp);
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
				                    	mNewDatabasePath = mWorkingDirectory + fileName + ".db";
				                    	
				                    	mProgressDialog.setTitle("Saving");
				                  	    mProgressDialog.setMessage(String.format("Please wait while saving \"%s\"...", mNewDatabasePath));
				                  	    mProgressDialog.show();
				                    	
										RTABMapLib.save(); // send save event
										//disable gui actions
										mItemSave.setEnabled(false);
										mItemOpen.setEnabled(false);
										mItemPostProcessing.setEnabled(false);
										mItemExport.setEnabled(false);
										mItemDataRecorderMode.setEnabled(false);
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
							  mNewDatabasePath = mWorkingDirectory + fileName + ".db";
							  
							  mProgressDialog.setTitle("Saving");
				        	  mProgressDialog.setMessage(String.format("Please wait while saving \"%s\"...", mNewDatabasePath));
				        	  mProgressDialog.show();
							  
							  RTABMapLib.save(); // send save event
							  //disable gui actions
							  mItemSave.setEnabled(false);
							  mItemOpen.setEnabled(false);
							  mItemPostProcessing.setEnabled(false);
							  mItemExport.setEnabled(false);
							  mItemDataRecorderMode.setEnabled(false);
						  }
					  }
				  }
			  });
			  builder.show();
    	  }
    	  else
    	  {
    		  mProgressDialog.setTitle("Saving");
        	  mProgressDialog.setMessage(String.format("Please wait while updating \"%s\"...", mOpenedDatabasePath));
        	  mProgressDialog.show();
        	  
    		  RTABMapLib.save(); // send save event
    		  //disable gui actions
			  mItemSave.setEnabled(false);
			  mItemOpen.setEnabled(false);
			  mItemPostProcessing.setEnabled(false);
			  mItemExport.setEnabled(false);
			  mItemDataRecorderMode.setEnabled(false);
    	  }
      }
      else if(itemId == R.id.reset)
      {
    	  ((TextView)findViewById(R.id.points)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.polygons)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.nodes)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.words)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
    	  ((TextView)findViewById(R.id.db_size)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.inliers)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.features)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.update_time)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.hypothesis)).setText(String.valueOf(0));
    	  ((TextView)findViewById(R.id.fps)).setText(String.valueOf(0));
		  mTotalLoopClosures = 0;
		  ((TextView)findViewById(R.id.total_loop)).setText(String.valueOf(mTotalLoopClosures));
    	  
		  if(mOpenedDatabasePath.isEmpty())
		  {
			  RTABMapLib.resetMapping();
		  }
		  else
		  {
			  mOpenedDatabasePath = "";
        	  RTABMapLib.openDatabase(mTempDatabasePath);
		  }
		  mMapIsEmpty = true;
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
            	  ((TextView)findViewById(R.id.memory)).setText(String.valueOf(Debug.getNativeHeapAllocatedSize()/(1024*1024)));
            	  ((TextView)findViewById(R.id.db_size)).setText(String.valueOf(0));
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
        		  RTABMapLib.openDatabase(mTempDatabasePath);
        		  
        		  mItemOpen.setEnabled(!mItemDataRecorderMode.isChecked() && mItemPause.isChecked());
    			  mItemPostProcessing.setEnabled(!mItemDataRecorderMode.isChecked() && mItemPause.isChecked());
    			  mItemExport.setEnabled(!mItemDataRecorderMode.isChecked() && mItemPause.isChecked());
            	  
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
      else if(itemId == R.id.export_obj || itemId == R.id.export_ply)
      {
    	  final String extension = itemId == R.id.export_ply ? ".ply" : ".obj";
    	  final boolean isOBJ = itemId == R.id.export_obj;
    	  
    	  final int polygons = Integer.parseInt(((TextView)findViewById(R.id.polygons)).getText().toString());
    	  
    	  AlertDialog.Builder builder = new AlertDialog.Builder(this);
		  builder.setTitle(String.format("File Name (*%s):", extension));
		  final EditText input = new EditText(this);
		  input.setInputType(InputType.TYPE_CLASS_TEXT);        
		  builder.setView(input);
		  String timeStamp = new SimpleDateFormat("yyMMdd-hhmmss").format(new Date());
		  input.setText(timeStamp);
		  input.setSelectAllOnFocus(true);
		  input.selectAll();
		  builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
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
			                    	final String path = mWorkingDirectory + fileName + extension;
			                    	
			                    	mItemExport.setEnabled(false);
			                    			                    	
			                    	mProgressDialog.setTitle("Exporting");
			                    	if(polygons > 1000000)
			                    	{
			                    		mProgressDialog.setMessage(String.format(
			                    				"Please wait while exporting \"%s\"...\n"
			                    				+ "Tip: With more than 1M polygons, to reduce exporting time and file size, consider:\n"
			                    				+ " a) increasing triangle size (Rendering options)\n"
			                    				+ " b) decreasing maximum camera depth (Rendering options)\n"
			                    				+ " c) activate Nodes Filtering (Mapping options)\n"
			                    				+ "then save/open to refresh the meshes.", fileName+extension));
			                    	}
			                    	else
			                    	{
			                    		mProgressDialog.setMessage(String.format("Please wait while exporting \"%s\"...", fileName+extension));
			                    	}
			                  	    
			                  	    mProgressDialog.show();
			                  	  
			                    	Thread exportThread = new Thread(new Runnable() {
			                		    public void run() {
			                		    	final boolean success = RTABMapLib.exportMesh(path);
			                		    	runOnUiThread(new Runnable() {
			                		    		public void run() {
					                		    	if(success)
							                    	{
					                		    		if(isOBJ)
					                		    		{	
					                		    			mToast.makeText(getActivity(), String.format("Mesh \"%s\" (with textures \"%s/\" and \"%s\") successfully exported!", path, fileName, fileName + ".mtl"), mToast.LENGTH_LONG).show();
						                		    	}
					                		    		else
					                		    		{
					                		    			mToast.makeText(getActivity(), String.format("Mesh \"%s\" successfully exported!", path), mToast.LENGTH_LONG).show();
					                		    		}
							                    	}
							                    	else
							                    	{
							                    		mToast.makeText(getActivity(), String.format("Exporting mesh to \"%s\" failed!", path), mToast.LENGTH_LONG).show();
							                    	}
					                		    	mItemExport.setEnabled(true);
					                		    	mProgressDialog.dismiss();
			                		    		}
			                		    	});
			                		    } 
			                		});
			                		exportThread.start();
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
						  final String path = mWorkingDirectory + fileName + extension;
						  mItemExport.setEnabled(false);
						  mProgressDialog.setTitle("Exporting");
	                  	  mProgressDialog.setMessage(String.format("Please wait while exporting \"%s\"...", fileName+extension));
	                  	  mProgressDialog.show();
						  Thread exportThread = new Thread(new Runnable() {
	                		    public void run() {
	                		    	final boolean success = RTABMapLib.exportMesh(path);
	                		    	runOnUiThread(new Runnable() {
	                		    		public void run() {
			                		    	if(success)
					                    	{
			                		    		if(isOBJ)
			                		    		{	
			                		    			mToast.makeText(getActivity(), String.format("Mesh \"%s\" (with textures \"%s/\" and \"%s\") successfully exported!", path, fileName, fileName + ".mtl"), mToast.LENGTH_LONG).show();
			                		    		}
			                		    		else
			                		    		{
			                		    			mToast.makeText(getActivity(), String.format("Mesh \"%s\" successfully exported!", path), mToast.LENGTH_LONG).show();
			                		    		}
					                    	}
					                    	else
					                    	{
					                    		mToast.makeText(getActivity(), String.format("Exporting mesh to \"%s\" failed!", path), mToast.LENGTH_LONG).show();
					                    	}
			                		    	mItemExport.setEnabled(true);
			                		    	mProgressDialog.dismiss();
	                		    		}
	                		    	});
	                		    } 
	                		});
	                		exportThread.start();
					  }
				  }
			  }
		  });
		  builder.show();
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
    		  builder.setTitle("Choose your file");
              builder.setItems(filesWithSize, new DialogInterface.OnClickListener() {
                  public void onClick(DialogInterface dialog, int which) {
                	  mOpenedDatabasePath = mWorkingDirectory + files[which];
                	  
                	  if(!mItemTrajectoryMode.isChecked())
                	  {
	                	  mProgressDialog.setTitle("Loading");
	                	  mProgressDialog.setMessage(String.format("Database \"%s\" loaded. Please wait while creating point clouds and meshes...", files[which]));
	                	  mProgressDialog.show();
                	  }

                	  RTABMapLib.openDatabase(mOpenedDatabasePath);
                	  RTABMapLib.setCamera(1);
                	  
                	  File extStore = new File(mTempDatabasePath);
                  	  if(extStore.exists())
                  	  {
                          extStore.delete();
                  	  }  
                  }
              });
              builder.show();
    	  }   	
      }
      else if(itemId == R.id.about)
      {
    	  AboutDialog about = new AboutDialog(this);
    	  about.setTitle("About RTAB-Map");
    	  about.show();
      }

      return true;
  }
}
