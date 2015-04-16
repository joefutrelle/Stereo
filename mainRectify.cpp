//===============================================================================//
//                                                                               //
// This program rectifies joined pairs of stereo images based on a list of files //
// where each line contains in input file followed by an output file that are    //
// separated by a comma, and them writes them to the specifed destination.       //
// Then rectified images are used to create a disparity map which in turn is     //
// used to create a 3D point cloud.                                              //
//                                                                               //
// Command line argument:   Name of parameter file that has pointers to camera   //
//                          the calibration matrices and the file list described //
//                          above.                                               //
// Author:                  Peter Honig, phonig@whoi.edu, March 1 2015           //
//                                                                               //
//===============================================================================//

#include "GlobalDefines.h"
#include "StereoStructDefines.h"	// needed for camera matrix and point cloud structs, as well as point cloud file enum
#include "AltitudeFromStereo.h"		// needed for doing the bulk of the computation
#include "FileIO.h"
#include "DataIO.h"					// needed for output of point cloud file
#include "demosaic.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	Parameters parameter;
	PointCloud pointCloud;
	CameraMatrix cameraMatrix;
	Mat image, imageRectified;

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
		cout << "ERROR in mainRectify: Parameter file path missing from argument list" << endl;
		return -1;
	}

	// validate and print the parameters for this run
	ValidateRuntimeParameters(parameter, RECTIFY);

	//*********************************************** NEEDED FOR COMPUTATION SECTION BELOW ****************************
	// get the intrinsic and extrinsic matrices from previous calibration
	if (!ReadCameraMatrices(parameter.calibrationDataDirectory, cameraMatrix))
	{
		cout << "Error in ReadCameraMatrices: Can't open/find the intrinsic/extrinsic matrix file" << endl;
		return -1;
	}
	//******************************************************************************************************************

	// get the list of images to be rectified
	vector<string> inputList, outputList;
	bool ok = ReadTwoImageListsFromFile(parameter.rectificationImageListFile, inputList, outputList);
	if(!ok || inputList.empty() || outputList.empty())
	{
		cout << "Error in ReadTwoImageListsFromFile: Cannot open the image list file" << parameter.rectificationImageListFile << " or the list is empty" << endl;
		return -1;
	}

	// process the images in the list of file names
	for (int i=0; i<(int)inputList.size(); i++)
	{
		// read the image to be processed
		Mat image = imread(inputList[i], CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
		if (image.empty())
		{
			cout << "Error in mainRectify: unable to either find or read image " << inputList[i] << endl;
			continue;
		}

		// check for tiff files and debayer if so (output is CV_U16C3 matrix type)
		unsigned found = (unsigned)inputList[i].find_last_of(".");
		if (inputList[i].substr(found+1) == "tif" || inputList[i].substr(found+1) == "tiff")
		{
			//cvtColor(image, image, CV_BayerBG2BGR);		// CV_BayerBG2BGR is RGGB
			image = demosaic(image, "RGGB");
		}

		//***************************** THIS SECTION IS WHERE WE DO ALL THE WORK ******************************************

		// input image and cameraMatrix, output imageRectified and pointCloud (and altitude of course)
		cout << "Computing rectification, point cloud and altitude " << i+1 << " of " << inputList.size() << endl;
		float altitude = AltitudeFromStereo(image, cameraMatrix, imageRectified, pointCloud,
			parameter.doNotRectify, parameter.displayRectifiedImage, parameter.displayDisparityImage, parameter.pauseForKeystroke);

		// check altitude for a valid range (1 to 3 1/2 meters perhaps?) and save data if okay
		if (altitude < 1000.0f || altitude > 3500.0f)
		{
			cout << "Invalid computed altitude. Skipping file " << inputList[i] << endl;
			continue;
		}
		else
		{
			// save the rectified image pair to disk
			if (!parameter.doNotRectify)
			{
				cout << "Saving rectified image pair" << endl;
				if (!imwrite(outputList[i], imageRectified))
					cout << endl << "ERROR in function WritePointCloud: Could not open/save " << outputList[i] << endl;
			}
			// save the point cloud to disk
			cout << "Saving point cloud" << endl;
			string filename = "C:/Users/Peterh~1/Desktop/PointCloud" + toString(i);	//*********** TEMPORARY UNTIL WE AGREE ON WHERE IT SHOULD GO *******
			if (!WritePointCloud(filename, pointCloud, imageRectified, PC_BINARY))
				cout << endl << "ERROR in function WritePointCloud: Could not open/save " << filename << endl;
		}
		//*****************************************************************************************************************
	}

	// all done
	return 0;
}
