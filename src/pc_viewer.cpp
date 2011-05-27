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
#include <iostream>

#include <XnCppWrapper.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/bind.hpp>

PCViewer::PCViewer(const XnChar* config_file)
    : viewer_("Point Cloud Viewer"),
      cloud_(new pcl::PointCloud <pcl::PointXYZ>) {

  cloud_->height = depth_md_.YRes();
  cloud_->width = depth_md_.XRes();
  cloud_->is_dense = true;

  cloud_->points.resize(cloud_->height * cloud_->width);

  z_scale_ = 1.0f / depth_md_.XRes();
  center_x_ = (cloud_->width >> 1 );
  center_y_ = (cloud_->height >> 1);
  bad_point_ = std::numeric_limits<float>::quiet_NaN ();
}

void PCViewer::run() {
  pcl::visualization::CloudViewer::VizCallable f = boost::bind(
      &PCViewer::display_cb_, this, _1);
  viewer_.runOnVisualizationThread(f);
  while (!viewer_.wasStopped());
}

void PCViewer::display_cb_(pcl::visualization::PCLVisualizer& viewer) {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = ni_context_.WaitAnyUpdateAll();
  CHECK_RC(rc, "Wait any update");

  depth_generator_.GetMetaData(depth_md_);

  int viewport0, viewport1;
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewport0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewport1);
  viewer.removePointCloud("cloud", viewport0);
  viewer.addPointCloud(convertToXYZPointCloud(depth_md_), "cloud", viewport0);
  if (depth_md_.FrameID() == 1)
    viewer.addPointCloud(convertToXYZPointCloud(depth_md_), "cloud", viewport1);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCViewer::convertToXYZPointCloud (
    const xn::DepthMetaData& depth) {
   // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
   register const XnDepthPixel* depth_map = depth.Data();

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


