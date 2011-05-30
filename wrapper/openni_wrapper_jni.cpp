/*
 * openni_wrapper_jni.cpp
 *
 *  Created on: May 30, 2011
 *      Author: yingyin
 */
#include "openni_wrapper_jni.h"
#include "openni_wrapper.h"

JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_initFromXmlFile
  (JNIEnv *env, jobject obj, jobject control_block, jstring config_file) {
  OpenNIWrapper** wrapper = (OpenNIWrapper**)env->GetDirectBufferAddress(cb);
  char strbuf[128];
  int len = env->GetStringLength(config_file);
  env->GetStringUTFRegion(config_file, 0, len, strbuf);
  *wrapper = new OpenNIWrapper(strbuf);
  return true;
}
