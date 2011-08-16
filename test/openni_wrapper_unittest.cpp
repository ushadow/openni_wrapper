#include "openni_wrapper.h"
#include "gtest/gtest.h"

class OpenNIWrapperTest : public ::testing::Test {
protected:
  OpenNIWrapper openni;

  virtual void SetUp() {
    openni.initFromXmlFile("/home/yingyin/workspace/tabletop/openni_wrapper/"
        "test/data/config.xml");
  }
};

TEST_F(OpenNIWrapperTest, ConvertDepthProjectiveToWorld) {
  float pts[] = {0, 0, 1500};
  openni.convertDepthProjectiveToWorld(pts);
  printf("(%f, %f, %f)\n", pts[0], pts[1], pts[2]);
}
