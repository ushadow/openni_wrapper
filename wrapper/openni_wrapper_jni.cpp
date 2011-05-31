/*
 * openni_wrapper_jni.cpp
 *
 *  Created on: May 30, 2011
 *      Author: yingyin
 */
#include "openni_wrapper_jni.h"
#include "openni_wrapper.h"

JNIEXPORT jboolean JNICALL Java_edu_mit_yingyin_tabletop_OpenNIWrapper_initFromXmlFile
  (JNIEnv *env, jobject obj, jobject ctrl_block, jstring config_file) {
  OpenNIWrapper** wrapper =
      (OpenNIWrapper**)env->GetDirectBufferAddress(ctrl_block);
  char strbuf[128];
  int len = env->GetStringLength(config_file);
  env->GetStringUTFRegion(config_file, 0, len, strbuf);
  *wrapper = new OpenNIWrapper();
  return (*wrapper)->initFromXmlFile(strbuf);
}
