package com.example.menghe.mygles;

import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class arTemplateActivity extends AppCompatActivity {
    private GLSurfaceView surfaceView;
    private boolean viewportChanged = false;
    private int viewportWidth;
    private int viewportHeight;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ar_template);
        setupSurfaceView();
        JniInterface.assetManager = getAssets();
    }
    private void setupSurfaceView(){
        surfaceView = (GLSurfaceView) findViewById(R.id.surfaceview);
        // Set up renderer.
        surfaceView.setPreserveEGLContextOnPause(true);
        surfaceView.setEGLContextClientVersion(2);
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0); // Alpha used for plane blending.
        surfaceView.setRenderer(new arTemplateActivity.Renderer());
        surfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    }
    private class Renderer implements GLSurfaceView.Renderer {
        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            GLES20.glClearColor(1.0f, .0f, .0f, 1.0f);
//        JniInterface.onGlSurfaceCreated(nativeApplication);
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
            viewportWidth = width;
            viewportHeight = height;
            viewportChanged = true;
            JniInterface.setupGrphicDraw(width, height);
        }

        @Override
        public void onDrawFrame(GL10 gl) {
            // Synchronized to avoid racing onDestroy.
            synchronized (this) {
//            if (nativeApplication == 0) {
//                return;
//            }
                if (viewportChanged) {
                    //int displayRotation = getWindowManager().getDefaultDisplay().getRotation();
//                JniInterface.onDisplayGeometryChanged(
//                        nativeApplication, displayRotation, viewportWidth, viewportHeight);
                    viewportChanged = false;
                }
//            JniInterface.onGlSurfaceDrawFrame(nativeApplication);
            }
            JniInterface.drawFrame();
        }
    }
}