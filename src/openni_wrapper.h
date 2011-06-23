#ifndef HAND_PROCESSOR_H_
#define HAND_PROCESSOR_H_

#include <string>
#include <XnCppWrapper.h>

class OpenNIWrapper {
public:
  OpenNIWrapper() {};
  bool initFromXmlFile(const XnChar* config_file);
  bool waitAnyUpdateAll();
  void getDepthMap(int* depth_buf);
  int depth_height() const;
  int depth_width() const;
  void cleanUp();
  void convertDepthProjectiveToWorld(float points[]);
private:
  xn::DepthGenerator depth_generator_;
  xn::DepthMetaData depth_md_;
  xn::Context ni_context_;
  XnUInt32 depth_height_, depth_width_;

  bool checkRC(XnStatus rc, const char* what);
  bool checkErrors(XnStatus rc, xn::EnumerationErrors& errors,
                   const char* what);
};

inline bool OpenNIWrapper::checkRC(XnStatus rc, const char* what) {
  bool success = true;
  if (rc != XN_STATUS_OK) {
    printf("%s failed: %s\n", what, xnGetStatusString(rc));
    success = false;
  }
  return success;
}

inline bool OpenNIWrapper::checkErrors(XnStatus rc,
                                       xn::EnumerationErrors& errors,
                                       const char* what) {
  bool success = true;
  if (rc == XN_STATUS_NO_NODE_PRESENT) {
    XnChar strError[1024];
    errors.ToString(strError, 1024);
    printf("%s failed: %s\n", what, strError);
    success = false;
  }
  return success;
}
#endif // HAND_PROCESSOR_H_
