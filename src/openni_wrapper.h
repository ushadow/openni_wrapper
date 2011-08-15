#ifndef HAND_PROCESSOR_H_
#define HAND_PROCESSOR_H_

#include <string>
#include <XnCppWrapper.h>
#include <XnOS.h>

class OpenNIWrapper {
public:
  /**
   * Loads a .raw file and reads the content into a buffer.
   *
   * Args:
   *    file_name: name of the file to load.
   *    buffer: pointer to an int array.
   *    buffer_size: size of the buffer.
   * Returns:
   *    True if loading is successful.
   */
  static bool loadFile(const XnChar* file_name, int* buffer,
                       const XnUInt32 buffer_size);
  OpenNIWrapper() {};
  bool initFromXmlFile(const XnChar* config_file);
  bool waitAnyUpdateAll();
  void getDepthMap(int* depth_buf);
  int depth_height() const;
  int depth_width() const;
  void cleanUp();
  void convertDepthProjectiveToWorld(float points[]);
private:
  static bool checkRC(XnStatus rc, const char* what);
  static bool checkErrors(XnStatus rc, xn::EnumerationErrors& errors,
                   const char* what);

  xn::DepthGenerator depth_generator_;
  xn::DepthMetaData depth_md_;
  xn::Context ni_context_;
  XnUInt32 depth_height_, depth_width_;
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

inline bool OpenNIWrapper::loadFile(const XnChar* file_name,
                                    int* buffer,
                                    const XnUInt32 buffer_size) {
  // Depth value is 16-bit.
  XnDepthPixel *depth_buffer = new XnDepthPixel[buffer_size];
  // The third parameter of xnOSLoadFile is number of bytes.
  XnStatus ret = xnOSLoadFile(file_name, depth_buffer, buffer_size * 2);
  if (!checkRC(ret, "Load file."))
    return false;
  for (int i = 0; i < buffer_size; i++)
    buffer[i] = depth_buffer[i];
  return true;
}

#endif // HAND_PROCESSOR_H_
