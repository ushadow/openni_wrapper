/*
 * simple_viewer.h
 *
 *  Created on: May 29, 2011
 *      Author: yingyin
 */

#ifndef SIMPLE_VIEWER_H_
#define SIMPLE_VIEWER_H_

#include <hand_processor.h>

class SimpleViewer {
public:
  SimpleViewer(HandProcessor* hand_processor);

private:
  boost::scoped_ptr<cv::Mat> frame_;
  HandProcessor* hand_processor_;
};

#endif /* SIMPLE_VIEWER_H_ */
