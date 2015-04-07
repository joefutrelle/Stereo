//===============================================================================//
//                                                                               //
// Header file for stereo camera calibration code                                //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#ifndef CalibrateStereoCamera_H_
#define CalibrateStereoCamera_H_

#include "GlobalDefines.h"

#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

using namespace cv;
using namespace std;

void CalibrateStereoCamera(const vector<string>& imageList, Size boardSize, float squareSize, string calibrationDataDirectory,
							bool displayImage=false, bool pauseForKeystroke=false);

#endif