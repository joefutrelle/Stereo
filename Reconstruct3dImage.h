//===============================================================================//
//                                                                               //
// Header for Reconstruct3dImage.cpp                                             //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 25 2015        //
//                                                                               //
//===============================================================================//

#ifndef Reconstruct3dImage_H_
#define Reconstruct3dImage_H_

#include "GlobalDefines.h"
#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

using namespace cv;
using namespace std;

PointCloud Reconstruct3dImage(Mat image, Mat Q, bool displayImage=false, bool pauseForKeystroke=false);
void triangle(Mat matrix, Point p1, Point p2, Point p3, Scalar fillValue);

#endif