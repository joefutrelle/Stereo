//===============================================================================//
//                                                                               //
// This program gets a list of chessboard target files and invokes the camera    //
// calibration function.                                                         //
// separated by a comma, ant them writes them to the specifed destination.       //
//                                                                               //
// Command line argument:   Name of paramter file that has pointers to camera    //
//                          the calibration chessboard file list.                //
// Variables:                                                                    //
//   parameter.displayImage       Boolean option to display chessboard corners   //
//   parameter.pauseForKeystroke  Boolean option to pause the chessboard display //
//                                                                               //
// Author:                  Peter Honig, phonig@whoi.edu, March 1 2015           //
//                                                                               //
//===============================================================================//

#include "GlobalDefines.h"
#include "CalibrateStereoCamera.h"
#include "FileIO.h"

#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Parameters parameter;
	// get user parameters from file
	if (argc >= 2)
	{
		if (!ReadRuntimeParameters(string(argv[1]), parameter))	// argv[1] is path to parameter file
		{
			cout << endl << " ERROR in function ReadRuntimeParameters: Could not open " << argv[1] << endl;
			return -1;
		}
	}
	else
	{
		cout << "ERROR in mainCalibrate: Parameter file path missing from argument list!" << endl;
		return -1;
	}

	// validate and print the parameters for this run
	ValidateRuntimeParameters(parameter, CALIBRATE);

	// get the list of calibration images (one line for left image, another for the right)
	vector<string> imageList, directoryList;
	bool ok = ReadImageListFromFile(parameter.calibrationImageListFile, directoryList, imageList, false);
	if(!ok || imageList.empty())
	{
		cout << "Error in mainCalibrate: Cannot open the image list file" << parameter.calibrationImageListFile << " or the list is empty" << endl;
		return -1;
	}

	// Calibrate the stereo camera rig from the list of chessboard calibration images
	CalibrateStereoCamera(imageList, Size(parameter.nHorizontal, parameter.nVertical), parameter.squareSize,
						parameter.calibrationDataDirectory, parameter.displayImage, parameter.pauseForKeystroke);
	
	return 0;
}
