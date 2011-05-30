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

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/bind.hpp>

#include <hand_processor.h>

PCViewer::PCViewer(HandProcessor* hand_processor) :
  viewer_("Point Cloud Viewer"), hand_processor_(hand_processor) {}

void PCViewer::run() {
  pcl::visualization::CloudViewer::VizCallable f = boost::bind(
      &PCViewer::display_cb_, this, _1);
  viewer_.runOnVisualizationThread(f);
  while (!viewer_.wasStopped());
}

void PCViewer::display_cb_(pcl::visualization::PCLVisualizer& viewer) {
  const xn::DepthMetaData* depth_md = hand_processor_->nextDepthMD();
  viewer.removePointCloud("cloud");
  viewer.addPointCloud(convertToXYZPointCloud(*depth_md), "cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0);
  std::stringstream ss;
  ss << "Frame: " << depth_md->FrameID();
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCViewer::convertToXYZPointCloud(
    const xn::DepthMetaData& depth_md) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // we have to use Data, since operator[] uses assert -> Debug-mode very slow!
  register const XnDepthPixel* depth_map = depth_md.Data();

  cloud->height = depth_md.YRes();
  cloud->width = depth_md.XRes();
  cloud->is_dense = true;

  cloud->points.resize(cloud->height * cloud->width);

  register float z_scale = 1.0f / depth_md.XRes();
  register float center_x = (cloud->width >> 1);
  register float center_y = (cloud->height >> 1);
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  register int depth_idx = 0;
  for (float v = -center_y; v < center_y; v += 1.0) {
    for (register float u = -center_x; u < center_x; u += 1.0, ++depth_idx) {
      register double z = depth_map[depth_idx] * 0.001;
      pcl::PointXYZ& pt = cloud->points[depth_idx];
      // Check for invalid measurements
      if (depth_map[depth_idx] == 0) {
        // not valid
        pt.x = pt.y = pt.z = bad_point;
        continue;
      }

      register double z_d = z * z_scale;
      // Fill in XYZ
      pt.x = u * z_d;
      pt.y = v * z_d;
      pt.z = -z;
    }
  }
  return cloud;
}

