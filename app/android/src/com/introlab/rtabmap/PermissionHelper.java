/*
 * Copyright 2018 Google Inc. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.introlab.rtabmap;

import android.Manifest;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.provider.Settings;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;

/** Helper to ask camera permission. */
public class PermissionHelper {
	public static final int CAMERA_CODE = 0;
	public static final int READ_EXTERNAL_STORAGE_CODE = 1;
	public static final int WRITE_EXTERNAL_STORAGE_CODE = 2;
	public static final int INTERNET_CODE = 3;
	public static final int ACCESS_NETWORK_STATE_CODE = 4;
	public static final int ACCESS_FINE_LOCATION_CODE = 5;
	public static final int ACCESS_WIFI_STATE_CODE = 6;

	/** Check to see we have the necessary permissions for this app. */
	public static boolean hasPermission(Activity activity, String permission) {
		return ContextCompat.checkSelfPermission(activity, permission) == PackageManager.PERMISSION_GRANTED;
	}

	/** Check to see we have the necessary permissions for this app, and ask for them if we don't. */
	public static void requestPermission(Activity activity, String permission) {
		int requestCode = -1;
		if (permission == Manifest.permission.CAMERA) {
			requestCode = CAMERA_CODE;
		} else if(permission == Manifest.permission.READ_EXTERNAL_STORAGE) {
			requestCode = READ_EXTERNAL_STORAGE_CODE;
		} else if(permission == Manifest.permission.WRITE_EXTERNAL_STORAGE) {
			requestCode = WRITE_EXTERNAL_STORAGE_CODE;
		} else if(permission == Manifest.permission.INTERNET) {
			requestCode = INTERNET_CODE;
		} else if(permission == Manifest.permission.ACCESS_NETWORK_STATE) {
			requestCode = ACCESS_NETWORK_STATE_CODE;
		} else if(permission == Manifest.permission.ACCESS_FINE_LOCATION) {
			requestCode = ACCESS_FINE_LOCATION_CODE;
		} else if(permission == Manifest.permission.ACCESS_WIFI_STATE) {
			requestCode = ACCESS_WIFI_STATE_CODE;
		}
		if(requestCode >=0)
		{
			ActivityCompat.requestPermissions(
					activity, new String[] {permission}, requestCode);
		}
	}

	/** Check to see if we need to show the rationale for this permission. */
	public static boolean shouldShowRequestPermissionRationale(Activity activity, String permission) {
		return ActivityCompat.shouldShowRequestPermissionRationale(activity, permission);
	}

	/** Launch Application Setting to grant permission. */
	public static void launchPermissionSettings(Activity activity) {
		Intent intent = new Intent();
		intent.setAction(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
		intent.setData(Uri.fromParts("package", activity.getPackageName(), null));
		activity.startActivity(intent);
	}
}
