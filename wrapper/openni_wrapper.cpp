/*
 * hand_processor.cpp
 *
 *  Created on: May 27, 2011
 *      Author: yingyin
 */
#include <openni_wrapper.h>
#include <stdlib.h>

bool OpenNIWrapper::initFromXmlFile(const XnChar* config_file) {
  XnStatus rc;

  xn::EnumerationErrors errors;
  printf("file = %s\n", config_file);
  rc = ni_context_.InitFromXmlFile(config_file, &errors);
  if (!checkErrors(rc, errors, "InitFromXmlFile"))
    return false;
  printf("rc = %u\n", rc);
  if (!checkRC(rc, "InitFromXmlFile"))
    return false;

  rc = ni_context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
  if (!checkRC(rc, "Find depth generator"))
    return false;
  depth_generator_.GetMetaData(depth_md_);
  depth_height_ = depth_md_.YRes();
  depth_width_ = depth_md_.XRes();
  fflush(stdout);
  return true;
}

const xn::DepthMetaData* OpenNIWrapper::nextDepthMD() {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = ni_context_.WaitAnyUpdateAll();
  checkRC(rc, "Wait any update");

  depth_generator_.GetMetaData(depth_md_);
  if (depth_md_.FrameID() == 1)
    bg_md_.CopyFrom(depth_md_);

  register int depth_idx = 0;
  register XnDepthPixel* depth_map = depth_md_.WritableData();
  register const XnDepthPixel* bg_map = bg_md_.Data();
  for (XnUInt y = 0; y < depth_md_.YRes(); y++)
    for (XnUInt x = 0; x < depth_md_.XRes(); x++, depth_idx++) {
      if (bg_map[depth_idx] - depth_map[depth_idx] <= 0)
        depth_map[depth_idx] = 0;
    }
  return &depth_md_;
}

int OpenNIWrapper::depth_height() const {
  return depth_height_;
}

int OpenNIWrapper::depth_width() const {
  return depth_width_;
}
