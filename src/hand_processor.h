#ifndef HAND_PROCESSOR_H_
#define HAND_PROCESSOR_H_

#include <XnCppWrapper.h>
#include <pcl/visualization/cloud_viewer.h>

class HandProcessor {
public:
  HandProcessor(const XnChar* config_file);
  const xn::DepthMetaData* nextDepthMD();
  int depth_height() const;
  int depth_width() const;
private:
  xn::DepthGenerator depth_generator_;
  xn::DepthMetaData depth_md_, bg_md_;
  xn::Context ni_context_;
  XnUInt32 depth_height_, depth_width_;
};

#endif // HAND_PROCESSOR_H_
