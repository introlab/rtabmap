package com.introlab.rtabmap;


import android.graphics.ImageFormat;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import java.nio.ByteBuffer;

public class TOF_ImageReader implements ImageReader.OnImageAvailableListener {

    public int WIDTH;
    public int HEIGHT;
    public ImageReader imageReader;
    public int frameCount = 0;
    public long timestamp;
    
    // Looper handler thread.
    private HandlerThread backgroundThread;
    // Looper handler.
    private Handler backgroundHandler;

    public ByteBuffer depth16_raw;

    TOF_ImageReader(){
    }

    public void createImageReader(int width, int height){
        this.WIDTH = width;
        this.HEIGHT = height;
        this.imageReader =
                ImageReader.newInstance(
                        width,
                        height,
                        ImageFormat.DEPTH16,
                        2);
        this.imageReader.setOnImageAvailableListener(this, this.backgroundHandler);
    }

    // CPU image reader callback.
    @Override
    public void onImageAvailable(ImageReader imageReader) {
        Image image  = imageReader.acquireLatestImage();
        if (image == null) {
            Log.w("RTABMapActivity", "onImageAvailable: Skipping null image.");
            return;
        }
        else{
            if(image.getFormat() == ImageFormat.DEPTH16){
                this.timestamp = image.getTimestamp();
                depth16_raw = image.getPlanes()[0].getBuffer().asReadOnlyBuffer();
                // copy raw undecoded DEPTH16 format depth data to NativeBuffer
                frameCount++;
            }
            else{
                Log.w("RTABMapActivity", "onImageAvailable: depth image not in DEPTH16 format, skipping image");
            }
        }
        image.close();
    }
    
 // Start background handler thread, used to run callbacks without blocking UI thread.
    public void startBackgroundThread() {
        this.backgroundThread = new HandlerThread("DepthDecoderThread");
        this.backgroundThread.start();
        this.backgroundHandler = new Handler(backgroundThread.getLooper());
    }

    // Stop background handler thread.
    public void stopBackgroundThread() {
        if (this.backgroundThread != null) {
            this.backgroundThread.quitSafely();
            try {
                this.backgroundThread.join();
                this.backgroundThread = null;
                this.backgroundHandler = null;
            } catch (InterruptedException e) {
                Log.e("RTABMapActivity", "Interrupted while trying to join depth background handler thread", e);
            }
        }
    }

}

