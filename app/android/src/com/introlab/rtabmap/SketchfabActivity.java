package com.introlab.rtabmap;

import java.io.File;
import java.io.IOException;
import java.util.List;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.AsyncTask;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.text.Editable;
import android.text.SpannableString;
import android.text.TextWatcher;
import android.text.method.LinkMovementMethod;
import android.text.util.Linkify;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class SketchfabActivity extends Activity implements OnClickListener {
	
	private static final String AUTHORIZE_PATH	= "https://sketchfab.com/oauth2/authorize";
	private static final String CLIENT_ID		= "RXrIJYAwlTELpySsyM8TrK9r3kOGQ5Qjj9VVDIfV";
	private static final String REDIRECT_URI	= "https://introlab.github.io/rtabmap/oauth2_redirect";
	
	ProgressDialog mProgressDialog;
	
	private Dialog mAuthDialog;
	
	private String mAuthToken;
	private String mWorkingDirectory;
	
	EditText mFilename;
	EditText mDescription;
	EditText mTags;
	CheckBox mDraft;
	Button mButtonOk;
	
	private SketchfabActivity getActivity() {return this;}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_sketchfab);
		
		mFilename = (EditText)findViewById(R.id.editText_filename);
		mDescription = (EditText)findViewById(R.id.editText_description);
		mTags = (EditText)findViewById(R.id.editText_tags);
		mDraft = (CheckBox)findViewById(R.id.checkBox_draft);
		mButtonOk = (Button)findViewById(R.id.button_ok);
		
		mProgressDialog = new ProgressDialog(this);
		mProgressDialog.setCanceledOnTouchOutside(false);
		
		mAuthToken = getIntent().getExtras().getString(RTABMapActivity.RTABMAP_AUTH_TOKEN_KEY);
		mFilename.setText(getIntent().getExtras().getString(RTABMapActivity.RTABMAP_FILENAME_KEY));
		mWorkingDirectory = getIntent().getExtras().getString(RTABMapActivity.RTABMAP_WORKING_DIR_KEY);
		
		SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
		String tags = sharedPref.getString(getString(R.string.pref_key_tags), getString(R.string.pref_default_tags));
		if(tags.isEmpty())
		{
			tags = getString(R.string.pref_default_tags);
		}
		mTags.setText(tags);
		
		mButtonOk.setEnabled(mFilename.getText().toString().length()>0);
		mButtonOk.setOnClickListener(this);
		
		mFilename.addTextChangedListener(new TextWatcher() {

			   @Override
			   public void afterTextChanged(Editable s) {}

			   @Override    
			   public void beforeTextChanged(CharSequence s, int start,
			     int count, int after) {
			   }

			   @Override    
			   public void onTextChanged(CharSequence s, int start,
			     int before, int count) {
			      mButtonOk.setEnabled(s.length() != 0);
			   }
			  });
		
		mFilename.setSelectAllOnFocus(true);
		mFilename.requestFocus();
	}
	
	@Override
	public void onClick(View v) {
		// Handle button clicks.
		switch (v.getId()) {
		case R.id.button_ok:
			shareToSketchfab();
			break;
		default:
			return;
		}
	}
	
	private void shareToSketchfab()
	{
		if(!mTags.getText().toString().isEmpty())
		{
			SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(this);
			SharedPreferences.Editor editor = sharedPref.edit();
			editor.putString(getString(R.string.pref_key_tags), mTags.getText().toString());
			// Commit the edits!
			editor.commit();
		}
		
		authorizeAndPublish(mFilename.getText().toString());	
	}
	
	private boolean isNetworkAvailable() {
	    ConnectivityManager connectivityManager 
	          = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
	    NetworkInfo activeNetworkInfo = connectivityManager.getActiveNetworkInfo();
	    return activeNetworkInfo != null && activeNetworkInfo.isConnected();
	}
	
	private void authorizeAndPublish(final String fileName)
	{
		if(!isNetworkAvailable())
		{
			// Visualize the result?
			new AlertDialog.Builder(this)
			.setTitle("Sharing to Sketchfab...")
			.setMessage("Network is not available. Make sure you have internet before continuing.")
			.setPositiveButton("Try Again", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {
					authorizeAndPublish(fileName);
				}
			})
			.setNeutralButton("Abort", new DialogInterface.OnClickListener() {
				public void onClick(DialogInterface dialog, int which) {
				}
			})
			.show();
			return;
		}	

		// get token the first time
		if(mAuthToken == null)
		{
			Log.i(RTABMapActivity.TAG,"We don't have the token, get it!");

			WebView web;
			mAuthDialog = new Dialog(this);
			mAuthDialog.setContentView(R.layout.auth_dialog);
			web = (WebView)mAuthDialog.findViewById(R.id.webv);
			web.setWebContentsDebuggingEnabled(!RTABMapActivity.DISABLE_LOG);
			web.getSettings().setJavaScriptEnabled(true);
			String auth_url = AUTHORIZE_PATH+"?redirect_uri="+REDIRECT_URI+"&response_type=token&client_id="+CLIENT_ID;
			Log.i(RTABMapActivity.TAG, "Auhorize url="+auth_url);
			web.setWebViewClient(new WebViewClient() {

				boolean authComplete = false;

				@Override
				public void onPageFinished(WebView view, String url) {
					super.onPageFinished(view, url);

					//Log.i(TAG,"onPageFinished url="+url);
					if(url.contains("error=access_denied")){
						Log.e(RTABMapActivity.TAG, "ACCESS_DENIED_HERE");
						authComplete = true;
						Toast.makeText(getApplicationContext(), "Error Occured", Toast.LENGTH_SHORT).show();
						mAuthDialog.dismiss();
					}
					else if (url.startsWith(REDIRECT_URI) && url.contains("access_token") && authComplete != true) {
						//Log.i(TAG,"onPageFinished received token="+url);
						String[] sArray = url.split("access_token=");
						mAuthToken = (sArray[1].split("&token_type=Bearer"))[0];
						authComplete = true;
						
						mAuthDialog.dismiss();

						zipAndPublish(fileName);
					}
				}
			});
			mAuthDialog.show();
			mAuthDialog.setTitle("Authorize RTAB-Map");
			mAuthDialog.setCancelable(true);
			web.loadUrl(auth_url);
		}
		else
		{
			zipAndPublish(fileName);
		}
	}

	private void zipAndPublish(final String fileName)
	{		
		mProgressDialog.setTitle("Upload to Sketchfab");
		mProgressDialog.setMessage(String.format("Compressing the files..."));
		mProgressDialog.show();

		Thread workingThread = new Thread(new Runnable() {
			public void run() {
				try{
					
					File tmpDir = new File(mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR);
					tmpDir.mkdirs();
					String[] fileNames = Util.loadFileList(mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR, false);
					if(!RTABMapActivity.DISABLE_LOG) Log.i(RTABMapActivity.TAG, String.format("Deleting %d files in \"%s\"", fileNames.length, mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR));
					for(int i=0; i<fileNames.length; ++i)
					{
						File f = new File(mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR + "/" + fileNames[i]);
						if(f.delete())
						{
							if(!RTABMapActivity.DISABLE_LOG) Log.i(RTABMapActivity.TAG, String.format("Deleted \"%s\"", f.getPath()));
						}
						else
						{
							if(!RTABMapActivity.DISABLE_LOG) Log.i(RTABMapActivity.TAG, String.format("Failed deleting \"%s\"", f.getPath()));
						}
					}
					File exportDir = new File(mWorkingDirectory + RTABMapActivity.RTABMAP_EXPORT_DIR);
					exportDir.mkdirs();
					
					if(RTABMapLib.writeExportedMesh(mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR, RTABMapActivity.RTABMAP_TMP_FILENAME))
					{
						String[] files = new String[0];
						// verify if we have all files
				
						fileNames = Util.loadFileList(mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR, false);
						if(fileNames.length > 0)
						{
							files = new String[fileNames.length];
							for(int i=0; i<fileNames.length; ++i)
							{
								files[i] = mWorkingDirectory + RTABMapActivity.RTABMAP_TMP_DIR + "/" + fileNames[i];
							}
						}
						else
						{
							if(!RTABMapActivity.DISABLE_LOG) Log.i(RTABMapActivity.TAG, "Missing files!");
						}
				
						if(files.length > 0)
						{
							final String[] filesToZip = files;
							final String zipOutput = mWorkingDirectory+fileName+".zip";
							Util.zip(filesToZip, zipOutput);
							runOnUiThread(new Runnable() {
								public void run() {
									mProgressDialog.dismiss();

									File f = new File(zipOutput);

									// Continue?
									AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
									builder.setTitle("File(s) compressed and ready to upload!");
									
									final int fileSizeMB = (int)f.length()/(1024 * 1024);
									final int fileSizeKB = (int)f.length()/(1024);
									if(fileSizeMB == 0)
									{
										Log.i(RTABMapActivity.TAG, String.format("Zipped files = %d KB", fileSizeKB));
										builder.setMessage(String.format("Total size to upload = %d KB. Do you want to continue?\n\n", fileSizeKB));
									}
									else
									{
										Log.i(RTABMapActivity.TAG, String.format("Zipped files = %d MB", fileSizeMB));
										builder.setMessage(String.format("Total size to upload = %d MB. %sDo you want to continue?\n\n"
												+ "Tip: To reduce the model size, you can also look at the Settings->Exporting options.", fileSizeMB,
												fileSizeMB>=50?"Note that for size over 50 MB, a Sketchfab PRO account is required, otherwise the upload may fail. ":""));
									}

									
									builder.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
										public void onClick(DialogInterface dialog, int which) {
											mProgressDialog.setTitle("Upload to Sketchfab");
											if(fileSizeMB == 0)
											{
												mProgressDialog.setMessage(String.format("Uploading model \"%s\" (%d KB) to Sketchfab...", fileName, fileSizeKB));
											}
											else
											{
												mProgressDialog.setMessage(String.format("Uploading model \"%s\" (%d MB) to Sketchfab...", fileName, fileSizeMB));
											}
											mProgressDialog.show();
											new uploadToSketchfabTask().execute(zipOutput, fileName);
										}
									});
									builder.setNegativeButton("No", new DialogInterface.OnClickListener() {
										public void onClick(DialogInterface dialog, int which) {
											// do nothing...
										}
									});
									builder.show();
								}
							});
						}
					}
					else
					{
						runOnUiThread(new Runnable() {
							public void run() {
								mProgressDialog.dismiss();
								Toast.makeText(getActivity(), String.format("Failed writing files!"), Toast.LENGTH_LONG).show();
							}
						});
					}
				}
				catch(IOException ex) {
					Log.e(RTABMapActivity.TAG, "Failed to zip", ex);
				}
			}
		});

		workingThread.start();
	}
	
	private class uploadToSketchfabTask extends AsyncTask<String, Void, Void>
	{
		String mModelUri;
		String mModelFilePath;
		String error = "";
		String mFileName;

		protected void onPreExecute() {
			//display progress dialog.
		}

		@Override
		protected void onPostExecute(Void result) {
			
			mProgressDialog.dismiss();
			//Task you want to do on UIThread after completing Network operation
			//onPostExecute is called after doInBackground finishes its task.
			if(mModelFilePath!= null)
			{
				File f = new File(mModelFilePath);
				f.delete(); // cleanup

				// See on sketchfab?
				  final SpannableString s = new SpannableString(
				        "Model \"" + mFileName + "\" is now processing on Sketchfab! You can click "
				        + "on the link below to see it on Sketchfab.\n\nhttps://sketchfab.com/models/"+mModelUri);
				  Linkify.addLinks(s, Linkify.WEB_URLS);
				final AlertDialog d = new AlertDialog.Builder(getActivity())
				.setTitle("Upload finished!")
				.setCancelable(false)
				.setMessage(s)
				.setPositiveButton("Close", new DialogInterface.OnClickListener() {
					public void onClick(DialogInterface dialog, int which) {
						Intent resultIntent = new Intent();
						resultIntent.putExtra(RTABMapActivity.RTABMAP_AUTH_TOKEN_KEY, mAuthToken); 
						setResult(Activity.RESULT_OK, resultIntent);
						finish();
					}
				}).create();
				d.show();
				((TextView)d.findViewById(android.R.id.message)).setMovementMethod(LinkMovementMethod.getInstance());
			}
			else
			{
				Toast.makeText(getApplicationContext(), String.format("Upload failed! Error=\"%s\"", error), Toast.LENGTH_SHORT).show();
			}
		}

		@Override
		protected Void doInBackground(String... files) {
			String charset = "UTF-8";
			File uploadFile = new File(files[0]);
			mFileName = files[1];
			String requestURL = "https://api.sketchfab.com/v3/models";

			Log.i(RTABMapActivity.TAG, "Uploading " + files[0]);

			try {
				MultipartUtility multipart = new MultipartUtility(requestURL, mAuthToken, charset);

				multipart.addFormField("name", mFileName);
				multipart.addFormField("description", mDescription.getText().toString());
				multipart.addFormField("tags", mTags.getText().toString());
				multipart.addFormField("source", "RTAB-Map");
				multipart.addFormField("isPublished", mDraft.isChecked()?"false":"true");	
				multipart.addFilePart("modelFile", uploadFile);

				Log.i(RTABMapActivity.TAG, "Starting multipart request");
				List<String> response = multipart.finish();

				Log.i(RTABMapActivity.TAG, "SERVER REPLIED:");

				for (String line : response) {
					Log.i(RTABMapActivity.TAG, line);
					//{"uri":"https:\/\/api.sketchfab.com\/v3\/models\/XXXXXXXXX","uid":"XXXXXXXXXX"}
					if(line.contains("\"uid\":\""))
					{
						String[] sArray = line.split("\"uid\":\"");
						mModelUri = (sArray[1].split("\""))[0];
						mModelFilePath = files[0];

						//patch model for orientation
						/*HttpClient httpClient = new DefaultHttpClient(); 
						try {
							String patchURL = "https://api.sketchfab.com/v3/models/"+ mModelUri +"/options";
							HttpPatch request = new HttpPatch(patchURL);
							String json = 
											"{\n"+
												"uid: "+ mModelUri + "\n" +
												"shading: shadeless\n"+
												//"orientation:\n"+
												//"{\n"+
												//	"axis : [1, 0, 0]\n"+
												//	"angle : 0\n"+
												//"}\n"+
											"}";

							request.setHeader("Authorization", "Bearer " + mAuthToken);
							StringEntity params =new StringEntity(json, "UTF-8");
							params.setContentType("application/json");
							request.setEntity(params);
							HttpResponse responsePatch = httpClient.execute(request);
							int responseStatus = responsePatch.getStatusLine().getStatusCode();
							Log.i(RTABMapActivity.TAG, "get data responseStatus: " + responseStatus);

						}catch (Exception e) {
							Log.e(RTABMapActivity.TAG, "Error while patching model", e);
							error = e.getMessage();
						}*/
					}
				}    		  
			} catch (IOException ex) {
				Log.e(RTABMapActivity.TAG, "Error while uploading", ex);
				error = ex.getMessage();
			}
			return null;
		}
	}
}
