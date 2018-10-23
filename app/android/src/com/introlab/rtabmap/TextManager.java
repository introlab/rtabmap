//package ri.blog.opengl008;
package com.introlab.rtabmap;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Iterator;
import java.util.Vector;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Rect;
import android.graphics.Typeface;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.text.TextPaint;

public class TextManager {
	
	/* SHADER Text
	 * 
	 * This shader is for rendering 2D text textures straight from a texture
	 * Color and alpha blended.
	 * 
	 */
	public static final String vs_Text =
		"uniform mat4 uMVPMatrix;" +
		"attribute vec4 vPosition;" +
		"attribute vec2 a_texCoord;" +
		"varying vec2 v_texCoord;" +
	    "void main() {" +
	    "  gl_Position = uMVPMatrix * vPosition;" +
	    "  v_texCoord = a_texCoord;" +
	    "}";
	public static final String fs_Text =
	    "precision mediump float;" +
	    "uniform float uColor;" +
	    "varying vec2 v_texCoord;" +
        "uniform sampler2D s_texture;" +
	    "void main() {" +
	    "  gl_FragColor = texture2D( s_texture, v_texCoord );" +
	    "  gl_FragColor.rgb *= uColor;" +
	    "}";
	
	public static int sp_Text;
	
	public static final int RI_TEXT_TEXTURE_SIZE = 512; // 512
	public static final float RI_TEXT_HEIGHT_BASE = 32.0f;
	public static final char RI_TEXT_START = ' ';
	public static final char RI_TEXT_STOP = '\u00B0'+1;
	
	public float getMaxTextHeight() {return mTextHeight;}
	
	private float mUVWidth;
	private float mUVHeight;
	private float mTextHeight;
	private float mColor;
	
	private FloatBuffer vertexBuffer;
	private FloatBuffer textureBuffer;
	private ShortBuffer drawListBuffer;
	
	private float[] vecs;
	private float[] uvs;
	private short[] indices;
	
	private int index_vecs;
	private int index_indices;
	private int index_uvs;

	private int texturenr;
	private int[] mTextures;
	
	private float uniformscale = 1.0f;
	
	float[] mCharacterWidth;
	
	public TextManager(Context context)
	{
		// Create the arrays
		vecs = new float[3 * 10];
		uvs = new float[2 * 10];
		indices = new short[10];
		
		// init as 0 as default
		texturenr = 0;
		
		// Text shader
		int vshadert = TextManager.loadShader(GLES20.GL_VERTEX_SHADER, TextManager.vs_Text);
		int fshadert = TextManager.loadShader(GLES20.GL_FRAGMENT_SHADER, TextManager.fs_Text);

		TextManager.sp_Text = GLES20.glCreateProgram();
		GLES20.glAttachShader(TextManager.sp_Text, vshadert);
		GLES20.glAttachShader(TextManager.sp_Text, fshadert); 		// add the fragment shader to program
		GLES20.glLinkProgram(TextManager.sp_Text);                  // creates OpenGL ES program executables
		
		// Generate Textures, if more needed, alter these numbers.
		mTextures = new int[1];
		GLES20.glGenTextures(1, mTextures, 0);
		
		// Create an empty, mutable bitmap
		Bitmap bitmap = Bitmap.createBitmap(RI_TEXT_TEXTURE_SIZE, RI_TEXT_TEXTURE_SIZE, Bitmap.Config.ARGB_4444);
		// get a canvas to paint over the bitmap
		Canvas canvas = new Canvas(bitmap);
		bitmap.eraseColor(0);

		// Draw the text
		TextPaint textPaint = new TextPaint();
		textPaint.setTextSize(RI_TEXT_HEIGHT_BASE);
		textPaint.setColor(Color.WHITE);
		textPaint.setAntiAlias(true);
		textPaint.setTypeface(Typeface.create("Arial", Typeface.BOLD));
		
		// compute real maximum height
		mTextHeight=0.0f;
		for(char c=RI_TEXT_START; c<RI_TEXT_STOP; ++c)
		{
			Rect textBounds = new Rect();
			textPaint.getTextBounds(String.valueOf(c), 0, 1, textBounds);
			if(textBounds.height() + textBounds.bottom > mTextHeight)
			{
				mTextHeight = textBounds.height() + textBounds.bottom;
			}
		}
		mUVWidth = (float)RI_TEXT_HEIGHT_BASE/(float)RI_TEXT_TEXTURE_SIZE;
		mUVHeight = mTextHeight/(float)RI_TEXT_TEXTURE_SIZE;
		mColor = 1.0f;
		
		int colCount = RI_TEXT_TEXTURE_SIZE/(int)RI_TEXT_HEIGHT_BASE;
		mCharacterWidth = new float[RI_TEXT_STOP-RI_TEXT_START];
		int i=0;
		for(char c=RI_TEXT_START; c<RI_TEXT_STOP; ++c)
		{
			Rect textBounds = new Rect();
			textPaint.getTextBounds(String.valueOf(c), 0, 1, textBounds);
			canvas.drawText(String.valueOf(c), (i%colCount)*RI_TEXT_HEIGHT_BASE, (i/colCount) * mTextHeight + RI_TEXT_HEIGHT_BASE, textPaint);
			mCharacterWidth[i] = textPaint.measureText(String.valueOf(c));
			++i;
		}
		
		GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, mTextures[0]);
		GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
		
		//Clean up
		bitmap.recycle();
	}
	
	public void setTextureID(int val)
	{
		texturenr = val;
	}
	
	public static int loadShader(int type, String shaderCode){

	    // create a vertex shader type (GLES20.GL_VERTEX_SHADER)
	    // or a fragment shader type (GLES20.GL_FRAGMENT_SHADER)
	    int shader = GLES20.glCreateShader(type);

	    // add the source code to the shader and compile it
	    GLES20.glShaderSource(shader, shaderCode);
	    GLES20.glCompileShader(shader);
	    
	    // return the shader
	    return shader;
	}
	
	
	public void AddCharRenderInformation(float[] vec, float[] cs, float[] uv, short[] indi)
	{
		// We need a base value because the object has indices related to 
		// that object and not to this collection so basicly we need to 
		// translate the indices to align with the vertexlocation in ou
		// vecs array of vectors.
		short base = (short) (index_vecs / 3);
			
		// We should add the vec, translating the indices to our saved vector
		for(int i=0;i<vec.length;i++)
		{
			vecs[index_vecs] = vec[i];
			index_vecs++;
		}
		
		// We should add the uvs
		for(int i=0;i<uv.length;i++)
		{
			uvs[index_uvs] = uv[i];
			index_uvs++;
		}

		// We handle the indices
		for(int j=0;j<indi.length;j++)
		{
			indices[index_indices] = (short) (base + indi[j]);
			index_indices++;
		}
	}

	public void PrepareDrawInfo(Vector<TextObject> txtcollection)
	{
		// Reset the indices.
		index_vecs = 0;
		index_indices = 0;
		index_uvs = 0;
		
		// Get the total amount of characters
		int charcount = 0;
		for (TextObject txt : txtcollection) {
			if(txt!=null)
			{
				if(!(txt.text==null))
				{
					charcount += txt.text.length(); 
				}
			}
		}
		
		// Create the arrays we need with the correct size.
		vecs = null;
		uvs = null;
		indices = null;
		
		vecs = new float[charcount * 12];
		uvs = new float[charcount * 8];
		indices = new short[charcount * 6];

	}
	
	public void PrepareDraw(Vector<TextObject> txtcollection)
	{
		// Setup all the arrays
		PrepareDrawInfo(txtcollection);
		
		// Using the iterator protects for problems with concurrency
		for( Iterator< TextObject > it = txtcollection.iterator(); it.hasNext() ; )
	    {
	    	TextObject txt = it.next();
	    	if(txt!=null)
			{
		    	if(!(txt.text==null))
				{
					convertTextToTriangleInfo(txt);
				}
			}
	    }
	}
	
	public void Draw(float[] m)
	{
		if(vecs.length > 0)
		{
			GLES20.glDisable(GLES20.GL_DEPTH_TEST);
			GLES20.glEnable(GLES20.GL_BLEND);
			
			// Set the correct shader for our grid object.
			GLES20.glUseProgram(sp_Text);
			
			// The vertex buffer.
			ByteBuffer bb = ByteBuffer.allocateDirect(vecs.length * 4);
			bb.order(ByteOrder.nativeOrder());
			vertexBuffer = bb.asFloatBuffer();
			vertexBuffer.put(vecs);
			vertexBuffer.position(0);
			
			// The texture buffer
			ByteBuffer bb2 = ByteBuffer.allocateDirect(uvs.length * 4);
			bb2.order(ByteOrder.nativeOrder());
			textureBuffer = bb2.asFloatBuffer();
			textureBuffer.put(uvs);
			textureBuffer.position(0);
			
			// initialize byte buffer for the draw list
			ByteBuffer dlb = ByteBuffer.allocateDirect(indices.length * 2);
			dlb.order(ByteOrder.nativeOrder());
			drawListBuffer = dlb.asShortBuffer();
			drawListBuffer.put(indices);
			drawListBuffer.position(0);
			
			// get handle to vertex shader's vPosition member
		    int mPositionHandle = GLES20.glGetAttribLocation(sp_Text, "vPosition");
		    
		    // Enable a handle to the triangle vertices
		    GLES20.glEnableVertexAttribArray(mPositionHandle);
		 
		    // Prepare the background coordinate data
		    GLES20.glVertexAttribPointer(mPositionHandle, 3,
		                                 GLES20.GL_FLOAT, false,
		                                 0, vertexBuffer);
		    
		    int mTexCoordLoc = GLES20.glGetAttribLocation(sp_Text, "a_texCoord" );
		    
		    // Prepare the texturecoordinates
		    GLES20.glVertexAttribPointer ( mTexCoordLoc, 2, GLES20.GL_FLOAT,
	                false, 
	                0, textureBuffer);
	
		    GLES20.glEnableVertexAttribArray ( mPositionHandle );
	        GLES20.glEnableVertexAttribArray ( mTexCoordLoc );
	
		    // get handle to shape's transformation matrix
	        int mtrxhandle = GLES20.glGetUniformLocation(sp_Text, "uMVPMatrix");
	        
	    	// Apply the projection and view transformation
	        GLES20.glUniformMatrix4fv(mtrxhandle, 1, false, m, 0);
	        
	        // get handle to color value
	        int colorhandle = GLES20.glGetUniformLocation(sp_Text, "uColor");
	        GLES20.glUniform1f(colorhandle, mColor);
	        
	        int mSamplerLoc = GLES20.glGetUniformLocation (sp_Text, "s_texture" );
	        
	        // Texture activate unit 0
	        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
			// Bind the texture to this unit.
	        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, mTextures[0]);
	        // Set the sampler texture unit to our selected id
		    GLES20.glUniform1i ( mSamplerLoc, texturenr);
		    
	        // Draw the triangle
	        GLES20.glDrawElements(GLES20.GL_TRIANGLES, indices.length, GLES20.GL_UNSIGNED_SHORT, drawListBuffer);
	
	        // Disable vertex array
	        GLES20.glDisableVertexAttribArray(mPositionHandle);
	        GLES20.glDisableVertexAttribArray(mTexCoordLoc);
		}
	}
	
	private void convertTextToTriangleInfo(TextObject val)
	{		
		// Get attributes from text object
		float x = val.x;
		float y = val.y;
		String text = val.text;
		
		// Create 
		for(int j=0; j<text.length(); j++)
		{
			// get ascii value
			char c = text.charAt(j);
			int c_val = (int)c;
			
			int indx = c_val-RI_TEXT_START;
			
			if(indx<0 || indx>=mCharacterWidth.length) {
				// unknown character, we will add a space for it to be save.
				indx = 0;
			}
			
			int colCount = RI_TEXT_TEXTURE_SIZE/(int)RI_TEXT_HEIGHT_BASE;
			
			// Calculate the uv parts
			int row = indx / colCount;
			int col = indx % colCount;
			
			float v = row * mUVHeight;
			float v2 = v + mUVHeight;
			float u = col * mUVWidth;
			float u2 = u + mCharacterWidth[indx]/(float)RI_TEXT_TEXTURE_SIZE;
			
			// Creating the triangle information
			float[] vec = new float[12];
			float[] uv = new float[8];
			float[] colors = new float[16];
			
			vec[0] = x;
			vec[1] = y + (mTextHeight * uniformscale);
			vec[2] = 0.99f;
			vec[3] = x;
			vec[4] = y;
			vec[5] = 0.99f;
			vec[6] = x + (mCharacterWidth[indx] * uniformscale);
			vec[7] = y;
			vec[8] = 0.99f;
			vec[9] = x + (mCharacterWidth[indx] * uniformscale);
			vec[10] = y + (mTextHeight * uniformscale);
			vec[11] = 0.99f;
		
			colors = new float[]
				    	{val.color[0], val.color[1], val.color[2], val.color[3],
						val.color[0], val.color[1], val.color[2], val.color[3],
						val.color[0], val.color[1], val.color[2], val.color[3],
						val.color[0], val.color[1], val.color[2], val.color[3]
				    			  		           };
			// 0.001f = texture bleeding hack/fix
			uv[0] = u+0.001f;
			uv[1] = v+0.001f;
			uv[2] = u+0.001f;
			uv[3] = v2-0.001f;
			uv[4] = u2-0.001f;
			uv[5] = v2-0.001f;
			uv[6] = u2-0.001f;
			uv[7] = v+0.001f;
			
			short[] inds = {0, 1, 2, 0, 2, 3};
			
			// Add our triangle information to our collection for 1 render call.
			AddCharRenderInformation(vec, colors, uv, inds);
			
			// Calculate the new position
			x += (mCharacterWidth[indx]  * uniformscale);
		}
	}

	public float getUniformscale() {
		return uniformscale;
	}

	public void setUniformscale(float uniformscale) {
		this.uniformscale = uniformscale;
	}
	
	public void setColor(float color) {
		mColor = color;
	}
}
