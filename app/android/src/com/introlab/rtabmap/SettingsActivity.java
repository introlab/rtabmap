package com.introlab.rtabmap;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Map.Entry;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.os.Bundle;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceActivity;
import android.text.InputType;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;

public class SettingsActivity extends PreferenceActivity implements OnSharedPreferenceChangeListener {
	
	private SettingsActivity getActivity() {return this;}
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        addPreferencesFromResource(R.layout.activity_settings);
        
        Preference buttonReset = findPreference(getString(R.string.pref_key_reset_button));
        buttonReset.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
        	@Override
        	public boolean onPreferenceClick(Preference preference) {   
        		getPreferenceScreen().getSharedPreferences().edit().clear().commit();
        		
                recreate();
        		
        		return true;
        	}
        });
        
        Preference buttonOpen = findPreference(getString(R.string.pref_key_open_button));
        buttonOpen.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
        	@Override
        	public boolean onPreferenceClick(Preference preference) {   
        		File prefsdir = new File(getApplicationInfo().dataDir,"shared_prefs");
        		if(prefsdir.exists() && prefsdir.isDirectory()){
        			ArrayList<String> filesArray = new ArrayList<String>(Arrays.asList(prefsdir.list()));
        			ArrayList<String> newList = new ArrayList<String>();
        			filesArray.remove("com.introlab.rtabmap_preferences.xml");
				filesArray.remove("WebViewChromiumPrefs.xml");
        			for (String s : filesArray) {
        				newList.add(s.substring(0, s.length()-4)); // rip off the ".xml"
        			}
        			final String[] files = newList.toArray(new String[filesArray.size()]);
        			if(files.length > 0)
        			{
        				AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        				builder.setTitle("Choose Presets:");
        				builder.setItems(files, new DialogInterface.OnClickListener() {
        					public void onClick(DialogInterface dialog, final int which) {
        						//sp1 is the shared pref to copy to
        						SharedPreferences.Editor ed = getPreferenceScreen().getSharedPreferences().edit(); 
        						SharedPreferences sp = getActivity().getSharedPreferences(files[which], MODE_PRIVATE); //The shared preferences to copy from
        						//Cycle through all the entries in the sp
        						for(Entry<String,?> entry : sp.getAll().entrySet()){ 
        						 Object v = entry.getValue(); 
        						 String key = entry.getKey();
        						 //Now we just figure out what type it is, so we can copy it.
        						 // Note that i am using Boolean and Integer instead of boolean and int.
        						 // That's because the Entry class can only hold objects and int and boolean are primatives.
        						 if(v instanceof Boolean) 
        						 // Also note that i have to cast the object to a Boolean 
        						 // and then use .booleanValue to get the boolean
        						    ed.putBoolean(key, ((Boolean)v).booleanValue());
        						 else if(v instanceof Float)
        						    ed.putFloat(key, ((Float)v).floatValue());
        						 else if(v instanceof Integer)
        						    ed.putInt(key, ((Integer)v).intValue());
        						 else if(v instanceof Long)
        						    ed.putLong(key, ((Long)v).longValue());
        						 else if(v instanceof String)
        						    ed.putString(key, ((String)v));         
        						}
        						ed.commit(); //save it.
        						recreate();
        						return;
        					}
        				});
        				builder.show();
        			}   
        		}

        		return true;
        	}
        });
        
        Preference buttonSave = findPreference(getString(R.string.pref_key_save_button));
        buttonSave.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
        	@Override
        	public boolean onPreferenceClick(Preference preference) {   
        		AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
    			builder.setTitle("Save Presets:");
    			final EditText input = new EditText(getActivity());
    			input.setInputType(InputType.TYPE_CLASS_TEXT); 
    			input.setImeOptions(EditorInfo.IME_FLAG_NO_EXTRACT_UI);
    			builder.setView(input);
    			builder.setPositiveButton("OK", new DialogInterface.OnClickListener() {
    				@Override
    				public void onClick(DialogInterface dialog, int which)
    				{
    					final String fileName = input.getText().toString();  
    					dialog.dismiss();
    					if(!fileName.isEmpty())
    					{
    						File newFile = new File(getApplicationInfo().dataDir + "/shared_prefs/" + fileName + ".xml");
    						if(newFile.exists())
    						{
    							new AlertDialog.Builder(getActivity())
    							.setTitle("Presets Already Exist")
    							.setMessage("Do you want to overwrite the existing file?")
    							.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
    								public void onClick(DialogInterface dialog, int which) {
    									saveConfig(fileName);
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
    							saveConfig(fileName);
    						}
    					}
    				}
    			});
    			AlertDialog alertToShow = builder.create();
    			alertToShow.getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE);
    			alertToShow.show();

        		return true;
        	}
        });
        
        Preference buttonRemove = findPreference(getString(R.string.pref_key_remove_button));
        buttonRemove.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
        	@Override
        	public boolean onPreferenceClick(Preference preference) {   
        		File prefsdir = new File(getApplicationInfo().dataDir,"shared_prefs");
        		if(prefsdir.exists() && prefsdir.isDirectory()){
        			ArrayList<String> filesArray = new ArrayList<String>(Arrays.asList(prefsdir.list()));
        			ArrayList<String> newList = new ArrayList<String>();
        			filesArray.remove("com.introlab.rtabmap_preferences.xml");
				filesArray.remove("WebViewChromiumPrefs.xml");
        			for (String s : filesArray) {
        				newList.add(s.substring(0, s.length()-4)); // rip off the ".xml"
        			}
        			final String[] files = newList.toArray(new String[filesArray.size()]);
        			if(files.length > 0)
        			{
        				AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
        				builder.setTitle("Remove Presets:");
        				builder.setItems(files, new DialogInterface.OnClickListener() {
        					public void onClick(DialogInterface dialog, final int which) {
        						File file = new File(getApplicationInfo().dataDir + "/shared_prefs/" + files[which] + ".xml");
        						if(file.exists())
        						{
        							file.delete();
        						}
        						return;
        					}
        				});
        				builder.show();
        			}   
        		}

        		return true;
        	}
        });
        
        
        ((Preference)findPreference(getString(R.string.pref_key_density))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_density))).getEntry() + ") "+getString(R.string.pref_summary_density));
        ((Preference)findPreference(getString(R.string.pref_key_depth))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_depth))).getEntry() + ") "+getString(R.string.pref_summary_depth));
        ((Preference)findPreference(getString(R.string.pref_key_min_depth))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_min_depth))).getEntry() + ") "+getString(R.string.pref_summary_min_depth));
        ((Preference)findPreference(getString(R.string.pref_key_point_size))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_point_size))).getEntry() + ") "+getString(R.string.pref_summary_point_size));
        ((Preference)findPreference(getString(R.string.pref_key_angle))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_angle))).getEntry() + ") "+getString(R.string.pref_summary_angle));
        ((Preference)findPreference(getString(R.string.pref_key_triangle))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_triangle))).getEntry() + ") "+getString(R.string.pref_summary_triangle));
        ((Preference)findPreference(getString(R.string.pref_key_background_color))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_background_color))).getEntry() + ") "+getString(R.string.pref_summary_background_color));
        ((Preference)findPreference(getString(R.string.pref_key_rendering_texture_decimation))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_rendering_texture_decimation))).getEntry() + ") "+getString(R.string.pref_summary_rendering_texture_decimation));
          
        ((Preference)findPreference(getString(R.string.pref_key_update_rate))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_update_rate))).getEntry() + ") "+getString(R.string.pref_summary_update_rate));
        ((Preference)findPreference(getString(R.string.pref_key_max_speed))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_max_speed))).getEntry() + ") "+getString(R.string.pref_summary_max_speed));
        ((Preference)findPreference(getString(R.string.pref_key_time_thr))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_time_thr))).getEntry() + ") "+getString(R.string.pref_summary_time_thr));
        ((Preference)findPreference(getString(R.string.pref_key_mem_thr))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_mem_thr))).getEntry() + ") "+getString(R.string.pref_summary_mem_thr));
        ((Preference)findPreference(getString(R.string.pref_key_loop_thr))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_loop_thr))).getEntry() + ") "+getString(R.string.pref_summary_loop_thr));
        ((Preference)findPreference(getString(R.string.pref_key_sim_thr))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_sim_thr))).getEntry() + ") "+getString(R.string.pref_summary_sim_thr));
        ((Preference)findPreference(getString(R.string.pref_key_min_inliers))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_min_inliers))).getEntry() + ") "+getString(R.string.pref_summary_min_inliers));
        ((Preference)findPreference(getString(R.string.pref_key_opt_error))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_opt_error))).getEntry() + ") "+getString(R.string.pref_summary_opt_error));
        ((Preference)findPreference(getString(R.string.pref_key_features_voc))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_features_voc))).getEntry() + ") "+getString(R.string.pref_summary_features_voc));
        ((Preference)findPreference(getString(R.string.pref_key_features))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_features))).getEntry() + ") "+getString(R.string.pref_summary_features));
        ((Preference)findPreference(getString(R.string.pref_key_features_type))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_features_type))).getEntry() + ") "+getString(R.string.pref_summary_features_type));
        ((Preference)findPreference(getString(R.string.pref_key_optimizer))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_optimizer))).getEntry() + ") "+getString(R.string.pref_summary_optimizer));
        ((Preference)findPreference(getString(R.string.pref_key_marker_detection))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_marker_detection))).getEntry() + ") "+getString(R.string.pref_summary_marker_detection));
        ((Preference)findPreference(getString(R.string.pref_key_marker_detection_depth_error))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_marker_detection_depth_error))).getEntry() + ") "+getString(R.string.pref_summary_marker_detection_depth_error));
        
        ((Preference)findPreference(getString(R.string.pref_key_cloud_voxel))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_cloud_voxel))).getEntry() + ") "+getString(R.string.pref_summary_cloud_voxel));
        ((Preference)findPreference(getString(R.string.pref_key_texture_size))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_texture_size))).getEntry() + ") "+getString(R.string.pref_summary_texture_size));
        ((Preference)findPreference(getString(R.string.pref_key_texture_count))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_texture_count))).getEntry() + ") "+getString(R.string.pref_summary_texture_count));
        ((Preference)findPreference(getString(R.string.pref_key_normal_k))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_normal_k))).getEntry() + ") "+getString(R.string.pref_summary_normal_k));
        ((Preference)findPreference(getString(R.string.pref_key_max_texture_distance))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_max_texture_distance))).getEntry() + ") "+getString(R.string.pref_summary_max_texture_distance));
        ((Preference)findPreference(getString(R.string.pref_key_min_texture_cluster_size))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_min_texture_cluster_size))).getEntry() + ") "+getString(R.string.pref_summary_min_texture_cluster_size));

        ((Preference)findPreference(getString(R.string.pref_key_opt_depth))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_opt_depth))).getEntry() + ") "+getString(R.string.pref_summary_opt_depth));
        ((Preference)findPreference(getString(R.string.pref_key_opt_color_radius))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_opt_color_radius))).getEntry() + ") "+getString(R.string.pref_summary_opt_color_radius));
        ((Preference)findPreference(getString(R.string.pref_key_opt_min_cluster_size))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_opt_min_cluster_size))).getEntry() + ") "+getString(R.string.pref_summary_opt_min_cluster_size));

        ((Preference)findPreference(getString(R.string.pref_key_cluster_ratio))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_cluster_ratio))).getEntry() + ") "+getString(R.string.pref_summary_cluster_ratio));
        ((Preference)findPreference(getString(R.string.pref_key_gain_max_radius))).setSummary("("+((ListPreference)findPreference(getString(R.string.pref_key_gain_max_radius))).getEntry() + ") "+getString(R.string.pref_summary_gain_max_radius));
    }
    
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        Preference pref = findPreference(key);

        if (pref instanceof ListPreference) {
        	if(key.compareTo(getString(R.string.pref_key_density))==0) pref.setSummary("("+ ((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_density));
        	if(key.compareTo(getString(R.string.pref_key_depth))==0) 
    		{
    			pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_depth));
    			float maxDepth = Float.parseFloat(((ListPreference)pref).getValue());
    			float minDepth = Float.parseFloat(((ListPreference)findPreference(getString(R.string.pref_key_min_depth))).getValue());
    			if(maxDepth > 0.0f && maxDepth <= minDepth)
    			{
    				((ListPreference)findPreference(getString(R.string.pref_key_min_depth))).setValueIndex(0);
    			}
    		}
        	if(key.compareTo(getString(R.string.pref_key_min_depth))==0) 
        	{
        		pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_min_depth));
        		float maxDepth = Float.parseFloat(((ListPreference)findPreference(getString(R.string.pref_key_depth))).getValue());
        		float minDepth = Float.parseFloat(((ListPreference)pref).getValue());
        		if(minDepth >= maxDepth)
        		{
        			((ListPreference)findPreference(getString(R.string.pref_key_depth))).setValueIndex(0);
        		}
        	}
        	if(key.compareTo(getString(R.string.pref_key_point_size))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_point_size));
        	if(key.compareTo(getString(R.string.pref_key_angle))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_angle));
        	if(key.compareTo(getString(R.string.pref_key_triangle))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_triangle));
        	if(key.compareTo(getString(R.string.pref_key_background_color))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_background_color));
        	if(key.compareTo(getString(R.string.pref_key_rendering_texture_decimation))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_rendering_texture_decimation));
        	
        	if(key.compareTo(getString(R.string.pref_key_update_rate))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_update_rate));
        	if(key.compareTo(getString(R.string.pref_key_max_speed))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_max_speed));
        	if(key.compareTo(getString(R.string.pref_key_time_thr))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_time_thr));
        	if(key.compareTo(getString(R.string.pref_key_mem_thr))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_mem_thr));
        	if(key.compareTo(getString(R.string.pref_key_loop_thr))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_loop_thr));
        	if(key.compareTo(getString(R.string.pref_key_sim_thr))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_sim_thr));
        	if(key.compareTo(getString(R.string.pref_key_min_inliers))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_min_inliers));
        	if(key.compareTo(getString(R.string.pref_key_opt_error))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_opt_error));
        	if(key.compareTo(getString(R.string.pref_key_features_voc))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_features_voc));
        	if(key.compareTo(getString(R.string.pref_key_features))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_features));
        	if(key.compareTo(getString(R.string.pref_key_features_type))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_features_type));
        	if(key.compareTo(getString(R.string.pref_key_optimizer))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_optimizer));
        	if(key.compareTo(getString(R.string.pref_key_marker_detection))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_marker_detection));
        	if(key.compareTo(getString(R.string.pref_key_marker_detection_depth_error))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_marker_detection_depth_error));
        	
        	if(key.compareTo(getString(R.string.pref_key_cloud_voxel))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_cloud_voxel));
        	if(key.compareTo(getString(R.string.pref_key_texture_size))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_texture_size));
        	if(key.compareTo(getString(R.string.pref_key_texture_count))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_texture_count));
        	if(key.compareTo(getString(R.string.pref_key_normal_k))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_normal_k));
        	if(key.compareTo(getString(R.string.pref_key_max_texture_distance))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_max_texture_distance));
        	if(key.compareTo(getString(R.string.pref_key_min_texture_cluster_size))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_min_texture_cluster_size));

        	if(key.compareTo(getString(R.string.pref_key_opt_depth))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_opt_depth));
        	if(key.compareTo(getString(R.string.pref_key_opt_color_radius))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_opt_color_radius));
        	if(key.compareTo(getString(R.string.pref_key_opt_min_cluster_size))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_opt_min_cluster_size));
        	
        	if(key.compareTo(getString(R.string.pref_key_cluster_ratio))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_cluster_ratio));
        	if(key.compareTo(getString(R.string.pref_key_gain_max_radius))==0) pref.setSummary("("+((ListPreference)pref).getEntry() + ") "+getString(R.string.pref_summary_gain_max_radius));
        
        }
    }
    
    @Override
    protected void onResume() {
        super.onResume();
        // Set up a listener whenever a key changes
        getPreferenceScreen().getSharedPreferences()
                .registerOnSharedPreferenceChangeListener(this);
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Unregister the listener whenever a key changes
        getPreferenceScreen().getSharedPreferences()
                .unregisterOnSharedPreferenceChangeListener(this);
    }
    
    private void saveConfig(String fileName)
    {
    	//sp1 is the shared pref to copy to
		SharedPreferences.Editor ed = getActivity().getSharedPreferences(fileName, MODE_PRIVATE).edit(); 
		SharedPreferences sp = getPreferenceScreen().getSharedPreferences(); //The shared preferences to copy from
		ed.clear(); // This clears the one we are copying to, but you don't necessarily need to do that.
		//Cycle through all the entries in the sp
		for(Entry<String,?> entry : sp.getAll().entrySet()){ 
		 Object v = entry.getValue(); 
		 String key = entry.getKey();
		 //Now we just figure out what type it is, so we can copy it.
		 // Note that i am using Boolean and Integer instead of boolean and int.
		 // That's because the Entry class can only hold objects and int and boolean are primatives.
		 if(v instanceof Boolean) 
		 // Also note that i have to cast the object to a Boolean 
		 // and then use .booleanValue to get the boolean
		    ed.putBoolean(key, ((Boolean)v).booleanValue());
		 else if(v instanceof Float)
		    ed.putFloat(key, ((Float)v).floatValue());
		 else if(v instanceof Integer)
		    ed.putInt(key, ((Integer)v).intValue());
		 else if(v instanceof Long)
		    ed.putLong(key, ((Long)v).longValue());
		 else if(v instanceof String)
		    ed.putString(key, ((String)v));         
		}
		ed.commit(); //save it.
    }
}
