/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class edu_mit_yingyin_tabletop_OpenNIWrapper */

#ifndef _Included_edu_mit_yingyin_tabletop_OpenNIWrapper
#define _Included_edu_mit_yingyin_tabletop_OpenNIWrapper
#ifdef __cplusplus
extern "C" {
#endif
#undef edu_mit_yingyin_tabletop_OpenNIWrapper_DEPTH_WIDTH
#define edu_mit_yingyin_tabletop_OpenNIWrapper_DEPTH_WIDTH 640L
#undef edu_mit_yingyin_tabletop_OpenNIWrapper_DEPTH_HEIGHT
#define edu_mit_yingyin_tabletop_OpenNIWrapper_DEPTH_HEIGHT 480L
/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    loadFile
 * Signature: (Ljava/lang/String;Ljava/nio/IntBuffer;I)V
 */
JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_loadFile
  (JNIEnv *, jclass, jstring, jobject, jint);

/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    initFromXmlFile
 * Signature: (Ljava/nio/IntBuffer;Ljava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_initFromXmlFile
  (JNIEnv *, jobject, jobject, jstring);

/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    waitAnyUpdateAll
 * Signature: (Ljava/nio/IntBuffer;)Z
 */
JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_waitAnyUpdateAll
  (JNIEnv *, jobject, jobject);

/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    getDepthMap
 * Signature: (Ljava/nio/IntBuffer;Ljava/nio/IntBuffer;)V
 */
JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_getDepthMap
  (JNIEnv *, jobject, jobject, jobject);

/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    cleanUp
 * Signature: (Ljava/nio/IntBuffer;)V
 */
JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_cleanUp
  (JNIEnv *, jobject, jobject);

/*
 * Class:     edu_mit_yingyin_tabletop_OpenNIWrapper
 * Method:    convertDepthProjectiveToWorld
 * Signature: (Ljava/nio/IntBuffer;Ljava/nio/FloatBuffer;)V
 */
JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_convertDepthProjectiveToWorld
  (JNIEnv *, jobject, jobject, jobject);

#ifdef __cplusplus
}
#endif
#endif
