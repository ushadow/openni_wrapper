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
#include <XnOS.h>
#include <XnCppWrapper.h>

#include <math.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../config/config.xml"

#define CHECK_RC(rc, what)                      \
  if (rc != XN_STATUS_OK) {                               \
    printf("%s failed: %s\n", what, xnGetStatusString(rc));   \
    return rc;                          \
  }

#define CHECK_ERRORS(rc, errors, what)    \
  if (rc == XN_STATUS_NO_NODE_PRESENT) {                   \
    XnChar strError[1024];        \
    errors.ToString(strError, 1024);  \
    printf("%s\n", strError);     \
    return (rc);            \
  }

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_context;
xn::DepthGenerator g_depth;
xn::ImageGenerator g_image;
xn::DepthMetaData g_depthMD;
xn::ImageMetaData g_imageMD;
xn::DepthMetaData g_bgMD;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertToXYZPointCloud (
    const xn::DepthMetaData& depth) {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

   // TODO cloud->header.stamp = time;
   cloud->height       = depth.YRes();
   cloud->width        = depth.XRes();
   cloud->is_dense     = false;

   cloud->points.resize (cloud->height * cloud->width);

   register float constant = 1.0f / depth.XRes();

   cloud->header.frame_id = depth.FrameID();

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

pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(void) {
  XnStatus rc = XN_STATUS_OK;

  // Read a new frame
  rc = g_context.WaitAnyUpdateAll();
//  if (rc != XN_STATUS_OK) {
//    printf("Read failed: %s\n", xnGetStatusString(rc));
//    return NULL;
//  }

  g_depth.GetMetaData(g_depthMD);
  g_image.GetMetaData(g_imageMD);
  if (g_depthMD.FrameID() == 1) {
    g_bgMD.CopyFrom(g_depthMD);
  }

  XnDepthPixel* curr_depth = g_depthMD.WritableData();
  const XnDepthPixel* bg_depth = g_bgMD.Data();
//  for (XnUInt y = 0; y < g_depthMD.YRes(); ++y) {
//    for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++curr_depth, ++bg_depth) {
//      *curr_depth = *bg_depth - *curr_depth;
//    }
//  }

  return ConvertToXYZPointCloud(g_depthMD);
}

int main(int argc, char* argv[]) {
  XnStatus rc;

  xn::EnumerationErrors errors;
  rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXmlFile");

  rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
  CHECK_RC(rc, "Find depth generator");
  rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
  CHECK_RC(rc, "Find hands generator");

  g_depth.GetMetaData(g_depthMD);
  g_image.GetMetaData(g_imageMD);

  // Hybrid mode isn't supported in this sample
  if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes()
      != g_depthMD.FullYRes()) {
    printf("The device depth and image resolution must be equal!\n");
    return 1;
  }

  // RGB is the only image format supported.
  if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24) {
    printf("The device image format must be RGB24\n");
    return 1;
  }

  pcl::visualization::CloudViewer viewer("Kinect");
  while (true) {
    viewer.showCloud(GetCloud());
    if (viewer.wasStopped())
      break;
  }

  return 0;
}
