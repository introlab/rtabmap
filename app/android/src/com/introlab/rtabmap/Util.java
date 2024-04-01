package com.introlab.rtabmap;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
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

	public static String[] loadFileList(final String directory, final boolean databasesOnly) {
		File path = new File(directory); 
		String fileList[];
		try {
			path.mkdirs();
		}
		catch(SecurityException e) {
			Log.e(RTABMapActivity.TAG, "unable to write on the sd card " + e.toString());
		}
		if(path.exists()) {
			FilenameFilter filter = new FilenameFilter() {

				@Override
				public boolean accept(File dir, String filename) {
					File sel = new File(dir, filename);
					if(databasesOnly)
					{
						return filename.compareTo(RTABMapActivity.RTABMAP_TMP_DB) != 0 && filename.endsWith(".db");
					}
					else
					{
						return sel.isFile();
					}
				}

			};
			fileList = path.list(filter);
			Arrays.sort(fileList);
			List<String> fileListt = new ArrayList<String>(Arrays.asList(fileList));
			Collections.sort(fileListt, new Comparator<String>() {

		        @Override
		        public int compare(String filename1, String filename2) {
		        	File file1 = new File(directory+"/"+filename1);
		        	File file2 = new File(directory+"/"+filename2);
		            long k = file1.lastModified() - file2.lastModified();
		            if(k > 0){
		               return -1;
		            }else if(k == 0){
		               return 0;
		            }else{
		              return 1;
		           }
		        }
		    });
			fileListt.toArray(fileList);
			
		}
		else {
			fileList = new String[0];
		}
		return fileList;
	}
	
	/**
	 * https://stackoverflow.com/questions/6701948/efficient-way-to-compare-version-strings-in-java
	 * Compares two version strings. 
	 * 
	 * Use this instead of String.compareTo() for a non-lexicographical 
	 * comparison that works for version strings. e.g. "1.10".compareTo("1.6").
	 * 
	 * @note It does not work if "1.10" is supposed to be equal to "1.10.0".
	 * 
	 * @param str1 a string of ordinal numbers separated by decimal points. 
	 * @param str2 a string of ordinal numbers separated by decimal points.
	 * @return The result is a negative integer if str1 is _numerically_ less than str2. 
	 *         The result is a positive integer if str1 is _numerically_ greater than str2. 
	 *         The result is zero if the strings are _numerically_ equal.
	 */
	public static int versionCompare(String str1, String str2) {
	    String[] vals1 = str1.split("\\.");
	    String[] vals2 = str2.split("\\.");
	    int i = 0;
	    // set index to first non-equal ordinal or length of shortest version string
	    while (i < vals1.length && i < vals2.length && vals1[i].equals(vals2[i])) {
	      i++;
	    }
	    // compare first non-equal ordinal number
	    if (i < vals1.length && i < vals2.length) {
	        int diff = Integer.valueOf(vals1[i]).compareTo(Integer.valueOf(vals2[i]));
	        return Integer.signum(diff);
	    }
	    // the strings are equal or one string is a substring of the other
	    // e.g. "1.2.3" = "1.2.3" or "1.2.3" < "1.2.3.4"
	    return Integer.signum(vals1.length - vals2.length);
	}
	
}
