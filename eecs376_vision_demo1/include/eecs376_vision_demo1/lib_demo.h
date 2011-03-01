#ifndef _LIB_DEMO_H
#define _LIB_DEMO_H

#include <opencv/cv.h>

void blobfind(const cv::Mat& src, cv::Mat& out, cv::Point2i& vec);

void normalizeColors(const cv::Mat& src, cv::Mat& out);

#endif
