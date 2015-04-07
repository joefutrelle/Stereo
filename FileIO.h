//===============================================================================//
//                                                                               //
// Header file for both stereo camera calibration and image rectification code   //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#ifndef FileIO_H_
#define FileIO_H_

#include "GlobalDefines.h"
#include <opencv2/core/core.hpp>

//===============================================================================//
//                                                                               //
// Header for FileIO.cpp                                                         //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#include <string>
#include <vector>

using namespace cv;
using namespace std;

bool ReadImageListFromFile(string imageListFile, vector<string> &directoryList, vector<string> &imageList, bool split=true);
bool ReadTwoImageListsFromFile(string imageListFile, vector<string> &inputList, vector<string> &outputList);
bool ReadRuntimeParameters(string filePath, Parameters &parameter);
void ValidateRuntimeParameters(Parameters parameter, ApplicationMode applicationMode);
bool ReadIntrinsicMatrices(string calibrationDataDirectory, Mat &M1, Mat &D1, Mat &M2, Mat &D2);
bool ReadExtrinsicMatrices(string calibrationDataDirectory, Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q);
bool WritePointCloud(string filename, PointCloud pointCloud, Mat image, FileFormat fileFormat=BINARY);

#endif