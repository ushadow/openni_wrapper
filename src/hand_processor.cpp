/*
 * hand_processor.cpp
 *
 *  Created on: May 27, 2011
 *      Author: yingyin
 */
#include <hand_processor.h>

#define CHECK_RC(rc, what)                      \
  if (rc != XN_STATUS_OK) {                               \
    printf("%s failed: %s\n", what, xnGetStatusString(rc));   \
  }

#define CHECK_ERRORS(rc, errors, what)    \
  if (rc == XN_STATUS_NO_NODE_PRESENT) {                   \
    XnChar strError[1024];        \
    errors.ToString(strError, 1024);  \
    printf("%s\n", strError);     \
  }

HandProcessor::HandProcessor(const XnChar* config_file) {
  XnStatus rc;

  xn::EnumerationErrors errors;
  rc = ni_context_.InitFromXmlFile(config_file, &errors);
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXmlFile");

  rc = ni_context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
  CHECK_RC(rc, "Find depth generator");
  depth_generator_.GetMetaData(depth_md_);
}

void HandProcessor::nextDepthCloud() {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = ni_context_.WaitAnyUpdateAll();
  CHECK_RC(rc, "Wait any update");

  depth_generator_.GetMetaData(depth_md_);
}

void PCViewer::convertToXYZPointCloud (const xn::DepthMetaData& depth_md,
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud) {

  // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
  register const XnDepthPixel* depth_map = depth.Data();

  depth_cloud->height = depth_md.YRes();
  depth_cloud->width = depth_md.XRes();
  depth_cloud->is_dense = true;
  depth_cloud->points.resize(depth_cloud->height * depth_cloud->width);

  register int depth_idx = 0;
  for (float v = -center_y_; v < center_y_; v += 1.0) {
  for (register float u = -center_x_; u < center_x_; u += 1.0, ++depth_idx) {
   register double z = depth_map[depth_idx] * 0.001;
   pcl::PointXYZ& pt = cloud_->points[depth_idx];
   // Check for invalid measurements
   if (depth_map[depth_idx] == 0) {
     // not valid
     pt.x = pt.y = pt.z = bad_point_;
     continue;
   }

   register double z_d = z * z_scale_;
   // Fill in XYZ
   pt.x = u * z_d;
   pt.y = v * z_d;
   pt.z = -z;
  }
  }
  return cloud_;
}

