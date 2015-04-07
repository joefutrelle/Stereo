//===============================================================================//
//                                                                               //
// Header for RectifyImage.cpp                                                   //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#ifndef RectifyImage_H_
#define RectifyImage_H_

#include "GlobalDefines.h"
#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

using namespace cv;
using namespace std;

Mat RectifyImage(Mat image, Mat M1, Mat D1, Mat R1, Mat P1, Mat M2, Mat D2, Mat R2, Mat P2,
				bool displayImage=false, bool pauseForKeystroke=false);

#endif