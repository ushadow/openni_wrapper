/*
 * 3d_viewer.h
 *
 *  Created on: May 26, 2011
 *      Author: yingyin
 */

#ifndef PC_VIEWER_H_
#define PC_VIEWER_H_

#include <string>

#include <XnCppWrapper.h>
#include <pcl/visualization/cloud_viewer.h>

class PCViewer {
public:
  PCViewer(const XnChar* config_file);
  void run();
private:
  xn::DepthGenerator depth_generator_;
  xn::DepthMetaData depth_md_;
  xn::Context ni_context_;
  pcl::visualization::CloudViewer viewer_;

  void displayCloud(pcl::visualization::PCLVisualizer& viewer);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZPointCloud (
      const xn::DepthMetaData& depth);
};

#endif /* PC_VIEWER_H_ */
