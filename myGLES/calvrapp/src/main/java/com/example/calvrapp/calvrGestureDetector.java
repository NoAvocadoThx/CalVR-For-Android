package com.example.calvrapp;

import android.content.Context;
import android.view.GestureDetector;
import android.view.MotionEvent;

public class calvrGestureDetector{
    private GestureDetector singleDetector;
    private SimpleTwoFingerDoubleTapDetector multiDetector;

    calvrGestureDetector(Context context){
        singleDetector = new GestureDetector(context, new gestureListener());
        multiDetector = new SimpleTwoFingerDoubleTapDetector() {
            @Override
            public void onTwoFingerLongPress(float ex, float ey){
                JniInterfaceCalVR.JNIonSingleTouch(2, ex, ey);
            }
            
            @Override
            public void onTwoFingerSingleTap(float ex, float ey){

            }

            @Override
            public void onTwoFingerDoubleTap(float ex, float ey) {
                JniInterfaceCalVR.JNIonDoubleTouch(2, ex, ey);
            }
        };
    }
    public boolean onTouchEvent(MotionEvent event) {
        singleDetector.onTouchEvent(event);
        multiDetector.onTouchEvent(event);
        return true;
    }
}