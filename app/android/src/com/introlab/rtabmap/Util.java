package com.introlab.rtabmap;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import android.content.Context;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.util.Log;

public class Util {
	
	public static final int ZIP_BUFFER_SIZE = 1<<20; // 1MB

	public static void zip(String file, String zipFile) throws IOException {
		Log.i(RTABMapActivity.TAG, "Zipping " + file +" to " + zipFile);
		String[] files = new String[1];
		files[0] = file;
		zip(files, zipFile);
	}

	public static void zip(String[] files, String zipFile) throws IOException {
		Log.i(RTABMapActivity.TAG, "Zipping " + String.valueOf(files.length) +" files to " + zipFile);
		BufferedInputStream origin = null;
		ZipOutputStream out = new ZipOutputStream(new BufferedOutputStream(new FileOutputStream(zipFile)));
		try { 
			byte data[] = new byte[ZIP_BUFFER_SIZE];

			for (int i = 0; i < files.length; i++) {
				FileInputStream fi = new FileInputStream(files[i]);    
				origin = new BufferedInputStream(fi, ZIP_BUFFER_SIZE);
				try {
					ZipEntry entry = new ZipEntry(files[i].substring(files[i].lastIndexOf("/") + 1));
					out.putNextEntry(entry);
					int count;
					while ((count = origin.read(data, 0, ZIP_BUFFER_SIZE)) != -1) {
						out.write(data, 0, count);
					}
				}
				finally {
					origin.close();
				}
			}
		}
		finally {
			out.close();
		}
	}

	
}
