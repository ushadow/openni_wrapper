/*
 * main.cpp
 *
 *  Created on: May 27, 2011
 *      Author: yingyin
 */
#include <pc_viewer.h>

#define CONFIG_FILE "../config/config.xml"

int main(int argc, char* argv[]) {
  PCViewer viewer(CONFIG_FILE);
  viewer.run();
  return 0;
}
