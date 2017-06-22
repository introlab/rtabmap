package com.introlab.rtabmap;

import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.drawable.Drawable;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.SimpleAdapter;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;

public class DatabaseListArrayAdapter extends SimpleAdapter {
	LayoutInflater inflater;
	Context context;
	ArrayList<HashMap<String, String>> arrayList;
	int imageWidth;

	public DatabaseListArrayAdapter(Context context, ArrayList<HashMap<String, String>> data, int resource, String[] from, int[] to) {
		super(context, data, resource, from, to);
		this.context = context;
		this.arrayList = data;
		this.imageWidth = (int)context.getResources().getDimension(R.dimen.image_width);
		inflater.from(context);
	}

	@Override
	public View getView(final int position, View convertView, ViewGroup parent) {
		View view = super.getView(position, convertView, parent);
		ImageView imageView = (ImageView) view.findViewById(R.id.imageView);
		
		boolean imageSet = false;
		String path = this.arrayList.get(position).get("path");
		if(!path.isEmpty())
		{
			SQLiteDatabase db = null;
			try {
				db = SQLiteDatabase.openDatabase(path, null, SQLiteDatabase.OPEN_READONLY);
				
				// get version
				Cursor c1 = db.rawQuery("SELECT version FROM Admin", null);    
		        if(c1.moveToFirst()) {
		        	String version = c1.getString(c1.getColumnIndex("version"));
		        	Log.i(RTABMapActivity.TAG, "Version="+version);
		        	if(Util.versionCompare(version, "0.12.0") >= 0) {
		        		Cursor c2 = db.rawQuery("SELECT preview_image FROM Admin WHERE preview_image is not null", null); 
		        		if(c2.moveToFirst()) {
		        			Log.i(RTABMapActivity.TAG, "Found image preview for db " + path);

		        			byte[] bytes = c2.getBlob(c2.getColumnIndex("preview_image"));
		        			ByteArrayInputStream inputStream = new ByteArrayInputStream(bytes);
		        			Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
		        			imageView.setImageBitmap(bitmap);
		        			imageSet = true;
		        		}
		        		else {
		        			Log.i(RTABMapActivity.TAG, "Not found image preview for db " + path);
		        		}
		        	}
		        	else {
		        		Log.i(RTABMapActivity.TAG, "Too old database for preview image, path = " + path);
		        	}
		        }
		        else {
		        	Log.e(RTABMapActivity.TAG, "Failed getting version from database");
		        }
	
	        } catch (Exception e) {
	            Log.e(RTABMapActivity.TAG, e.getMessage());
	        }
			finally {
				if(db != null && db.isOpen()) {
	            	db.close();
	            }
			}
		}
		else
		{
			Log.e(RTABMapActivity.TAG, "Database path empty for item " + position);
		}
		
		if(!imageSet)
		{
			Drawable myDrawable = context.getResources().getDrawable(R.drawable.ic_launcher);
			imageView.setImageDrawable(myDrawable);
		}
		LinearLayout.LayoutParams layoutParams = new LinearLayout.LayoutParams(imageWidth,imageWidth/(!imageSet?2:1));
		imageView.setLayoutParams(layoutParams);
		return view;
	}

}
