//
// Created by menghe on 8/3/2018.
//

#ifndef MYGLES_JNI_INTERFACE_CALVR_H
#define MYGLES_JNI_INTERFACE_CALVR_H

#include <jni.h>



#define JNI_METHOD(returnType, funcName)\
    JNIEXPORT returnType JNICALL        \
        Java_com_example_menghe_mygles_JniInterfaceCalVR_##funcName

//define helper functions to provide access to Java from C via JNI
extern "C"{
JNI_METHOD(jlong, JNIcreateController)(JNIEnv* env, jclass, jobject asset_manager);

JNI_METHOD(void, JNIdrawFrame)(JNIEnv *, jclass);

JNI_METHOD(void, JNIonGlSurfaceCreated)(JNIEnv *, jclass, jstring calvr_path);

JNI_METHOD(void, JNIonResourceLoaded)(JNIEnv*, jclass, jstring path);

JNI_METHOD(void, JNIonViewChanged)(JNIEnv*, jclass, jint rotation, jint width, jint height);

JNIEnv * GetJniEnv();
}


#endif //MYGLES_JNI_INTERFACE_CALVR_H