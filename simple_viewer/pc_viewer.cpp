/****************************************************************************
 *                                                                           *
 *  OpenNI 1.1 Alpha                                                         *
 *  Copyright (C) 2011 PrimeSense Ltd.                                       *
 *                                                                           *
 *  This file is part of OpenNI.                                             *
 *                                                                           *
 *  OpenNI is free software: you can redistribute it and/or modify           *
 *  it under the terms of the GNU Lesser General Public License as published *
 *  by the Free Software Foundation, either version 3 of the License, or     *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  OpenNI is distributed in the hope that it will be useful,                *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU Lesser General Public License *
 *  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
 *                                                                           *
 ****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <pc_viewer.h>

#include <string.h>

#include <XnCppWrapper.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/bind.hpp>

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

PCViewer::PCViewer(const XnChar* config_file) : viewer_("Point Cloud Viewer") {
  XnStatus rc;

  xn::EnumerationErrors errors;
  rc = ni_context_.InitFromXmlFile(config_file, &errors);
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXmlFile");

  rc = ni_context_.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
  CHECK_RC(rc, "Find depth generator");
}

void PCViewer::run() {
  pcl::visualization::CloudViewer::VizCallable f = boost::bind(
      &PCViewer::displayCloud, this, _1);
  viewer_.runOnVisualizationThread(f);
  while (!viewer_.wasStopped());
}

void PCViewer::displayCloud(pcl::visualization::PCLVisualizer& viewer) {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = ni_context_.WaitAnyUpdateAll();
  CHECK_RC(rc, "Wait any update");

  depth_generator_.GetMetaData(depth_md_);

  viewer.removePointCloud();
  viewer.addPointCloud(convertToXYZPointCloud(depth_md_));
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCViewer::convertToXYZPointCloud (
    const xn::DepthMetaData& depth) {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
       (new pcl::PointCloud <pcl::PointXYZ>);

   cloud->height = depth.YRes();
   cloud->width = depth.XRes();
   cloud->header.frame_id = depth.FrameID();
   cloud->is_dense = true;

   cloud->points.resize(cloud->height * cloud->width);

   register float constant = 1.0f / depth.XRes();

   register float centerX = (cloud->width >> 1 );
   float centerY = (cloud->height >> 1);

   float bad_point = std::numeric_limits<float>::quiet_NaN ();

   // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
   register const XnDepthPixel* depth_map = depth.Data();

   register int depth_idx = 0;
   for (float v = -centerY; v < centerY; v += 1.0) {
     for (register float u = -centerX; u < centerX; u += 1.0, ++depth_idx) {
       register double z = depth_map[depth_idx] * 0.001;
       pcl::PointXYZ& pt = cloud->points[depth_idx];
       // Check for invalid measurements
       if (depth_map[depth_idx] == 0) {
         // not valid
         pt.x = pt.y = pt.z = bad_point;
         continue;
       }

       register double z_d = z * constant;
       // Fill in XYZ
       pt.x = u * z_d;
       pt.y = v * z_d;
       pt.z = z;
     }
   }
   return cloud;
 }


