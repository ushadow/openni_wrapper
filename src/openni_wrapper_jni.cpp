/*
 * openni_wrapper_jni.cpp
 *
 *  Created on: May 30, 2011
 *      Author: yingyin
 */
#include "openni_wrapper_jni.h"
#include "openni_wrapper.h"

JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_initFromXmlFile
  (JNIEnv *env, jobject obj, jobject ctrl_block, jstring config_file,
   jobject widthBuf, jobject heightBuf) {
  OpenNIWrapper::getenv();
  OpenNIWrapper** wrapper =
      (OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block);
  char strbuf[128];
  int len = env->GetStringLength(config_file);
  env->GetStringUTFRegion(config_file, 0, len, strbuf);
  *wrapper = new OpenNIWrapper();
  bool ret = (*wrapper)->initFromXmlFile(strbuf);
  int* width = (int*)env->GetDirectBufferAddress(widthBuf);
  *width = (*wrapper)->depth_width();
  int* height = (int*)env->GetDirectBufferAddress(heightBuf);
  *height = (*wrapper)->depth_height();
  return ret;
}

JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_waitAnyUpdateAll
  (JNIEnv *env, jobject obj, jobject ctrl_block) {
  OpenNIWrapper* wrapper =
      *((OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block));
  return wrapper->waitAnyUpdateAll();
}

JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_waitDepthUpdateAll
  (JNIEnv *env, jobject obj, jobject ctrl_block) {
  OpenNIWrapper* wrapper =
      *((OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block));
  return wrapper->waitDepthUpdateAll();
}

JNIEXPORT jint JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_getDepthMap
  (JNIEnv *env, jobject obj, jobject ctrl_block, jobject depth_buf) {
  OpenNIWrapper* wrapper =
      *((OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block));
  return wrapper->getDepthMap((int*)env->GetDirectBufferAddress(depth_buf));
}

JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_release
  (JNIEnv *env, jobject obj, jobject ctrl_block) {
  OpenNIWrapper* wrapper =
        *((OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block));
  wrapper->release();
  delete wrapper;
}

JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_convertDepthProjectiveToWorld
  (JNIEnv *env, jobject obj, jobject ctrl_block, jobject points) {
  OpenNIWrapper* wrapper =
      *((OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block));
  wrapper->convertDepthProjectiveToWorld(
      (float*)env->GetDirectBufferAddress(points));
}

JNIEXPORT void JNICALL Java_edu_mit_yingyin_tabletop_models_PartialOpenNIDevice_loadFile
  (JNIEnv *env, jclass c, jstring file_name, jobject buffer, jint size) {
  char strbuf[128];
  int len = env->GetStringLength(file_name);
  env->GetStringUTFRegion(file_name, 0, len, strbuf);
  OpenNIWrapper::loadFile(strbuf,
                          (int*)env->GetDirectBufferAddress(buffer),
                          size);
}
