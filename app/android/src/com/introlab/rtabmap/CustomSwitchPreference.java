package com.introlab.rtabmap;

import android.content.Context;
import android.preference.SwitchPreference;
import android.util.AttributeSet;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Switch;

/**
 * 
 * @author mathieu
 * Bug fix of switch preferences changing states 
 * when scrolling on Jelly Bean: 
 * https://issuetracker.google.com/issues/36941388#comment4
 */

public class CustomSwitchPreference extends SwitchPreference { 

    /** 
     * Construct a new SwitchPreference with the given style options.
     * 
     * @param context The Context that will style this preference 
     * @param attrs Style attributes that differ from the default 
     * @param defStyle Theme attribute defining the default style options 
     */ 
    public CustomSwitchPreference(Context context, AttributeSet attrs, int defStyle) { 
        super(context, attrs, defStyle); 
    } 

    /** 
     * Construct a new SwitchPreference with the given style options.
     * 
     * @param context The Context that will style this preference 
     * @param attrs Style attributes that differ from the default 
     */ 
    public CustomSwitchPreference(Context context, AttributeSet attrs) { 
        super(context, attrs); 
    } 

    /** 
     * Construct a new SwitchPreference with default style options. 
     * 
     * @param context The Context that will style this preference 
     */ 
    public CustomSwitchPreference(Context context) { 
        super(context, null); 
    } 

    @Override 
    protected void onBindView(View view) { 
        // Clean listener before invoke SwitchPreference.onBindView 
        ViewGroup viewGroup= (ViewGroup)view; 
        clearListenerInViewGroup(viewGroup); 
        super.onBindView(view); 
    } 

    /** 
     * Clear listener in Switch for specify ViewGroup. 
     * 
     * @param viewGroup The ViewGroup that will need to clear the listener. 
     */ 
    private void clearListenerInViewGroup(ViewGroup viewGroup) { 
        if (null == viewGroup) { 
            return; 
        } 

        int count = viewGroup.getChildCount(); 
        for(int n = 0; n < count; ++n) { 
            View childView = viewGroup.getChildAt(n); 
            if(childView instanceof Switch) { 
                final Switch switchView = (Switch) childView; 
                switchView.setOnCheckedChangeListener(null); 
                return; 
            } else if (childView instanceof ViewGroup){ 
                ViewGroup childGroup = (ViewGroup)childView; 
                clearListenerInViewGroup(childGroup); 
            } 
        } 
    } 

}