/*
 * hand_processor.cpp
 *
 *  Created on: May 27, 2011
 *      Author: yingyin
 */
#include <openni_wrapper.h>
#include <stdlib.h>

bool OpenNIWrapper::debug = false;

bool OpenNIWrapper::initFromXmlFile(const XnChar* config_file) {
  XnStatus rc;

  xn::EnumerationErrors errors;
  rc = ni_context_.InitFromXmlFile(config_file, &errors);
  if (!checkErrors(rc, errors, "InitFromXmlFile"))
    return false;
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

bool OpenNIWrapper::waitAnyUpdateAll() {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = ni_context_.WaitAnyUpdateAll();
  if (!checkRC(rc, "Wait any update"))
    return false;
  return true;
}

int OpenNIWrapper::getDepthMap(int* depth_buf) {
  depth_generator_.GetMetaData(depth_md_);
  register int depth_idx = 0;
  const XnDepthPixel* depth_map = depth_md_.Data();
  for (XnUInt y = 0; y < depth_md_.YRes(); y++) {
    for (XnUInt x = 0; x < depth_md_.XRes(); x++, depth_idx++) {
      depth_buf[depth_idx] = depth_map[depth_idx];
    }
  }
  int frameID = depth_md_.FrameID();
  if (debug) {
    printf("Frame ID = %d\n", frameID);
    fflush(stdout);
  }
  return frameID;
}

int OpenNIWrapper::depth_height() const {
  return depth_height_;
}

int OpenNIWrapper::depth_width() const {
  return depth_width_;
}

void OpenNIWrapper::cleanUp() {
  ni_context_.Shutdown();
  printf("OpenNI cleaned up.\n");
  fflush(stdout);
}

void OpenNIWrapper::convertDepthProjectiveToWorld(float points[]) {
  int num_points = sizeof(points) / 3;
  XnPoint3D *xn_pts = new XnPoint3D[num_points];
  for (int i = 0; i < num_points; i++)
    xn_pts[i] = {points[i * 3], points[i * 3 + 1], points[i * 3 + 2]};
  depth_generator_.ConvertProjectiveToRealWorld(num_points, xn_pts, xn_pts);
  for (int i = 0; i < num_points; i++) {
    points[i * 3] = xn_pts[i].X;
    points[i * 3 + 1] = xn_pts[i].Y;
    points[i * 3 + 2] = xn_pts[i].Z;
  }
  delete[] xn_pts;
}
