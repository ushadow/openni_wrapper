/*
 * main.cpp
 *
 *  Created on: May 27, 2011
 *      Author: yingyin
 */
#include <boost/scoped_ptr.hpp>
#include <pc_viewer.h>
#include <hand_processor.h>

#define CONFIG_FILE "../config/config.xml"

int main(int argc, char* argv[]) {
  boost::scoped_ptr<HandProcessor> hand_processor(
      new HandProcessor(CONFIG_FILE));
  PCViewer viewer(hand_processor.get());
  viewer.run();
  return 0;
}
