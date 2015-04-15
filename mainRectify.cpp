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
#include "RectifyImage.h"
#include "Reconstruct3dImage.h"
#include "FileIO.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
	Parameters parameter;
	PointCloud pointCloud;
	Mat image, imageRectified;
	Mat R, T, Q, M1, D1, R1, P1, M2, D2, R2, P2;

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

	// get the intrinsic matrices from previous calibration
	if (!ReadIntrinsicMatrices(parameter.calibrationDataDirectory, M1, D1, M2, D2))
	{
		cout << "Error in ReadIntrinsicMatrices: Can't open/find the intrinsic matrix file" << endl;
		return -1;
	}

	// get the extrinsic matrices from previous calibration
	if (!ReadExtrinsicMatrices(parameter.calibrationDataDirectory, R, T, R1, R2, P1, P2, Q))
	{
		cout << "Error in ReadExtrinsicMatrices: Can't open/find the extrinsic matrix file" << endl;
		return -1;
	}

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
			cvtColor(image, image, CV_BayerRG2BGR);

		// rectify the image pair
		if (parameter.doNotRectify)
		{
			cout << "Skipping rectification of image " << i+1 << " of " << inputList.size() << endl;
			imageRectified = image;
		}
		else
		{
			cout << "Rectifying image " << i+1 << " of " << inputList.size() << endl;
			imageRectified = RectifyImage(image, M1, D1, R1, P1, M2, D2, R2, P2, parameter.displayRectifiedImage, parameter.pauseForKeystroke);
			cout << "Saving rectified image pair" << endl;
			imwrite(outputList[i], imageRectified);
		}

		// generate point cloud
		cout << "Computing disparity image and point cloud" << endl;
		pointCloud = Reconstruct3dImage(imageRectified, Q, parameter.displayDisparityImage, parameter.pauseForKeystroke);

		// save the point cloud to disk
		cout << "Saving point cloud" << endl;
		string filename = "C:/Users/Peterh~1/Desktop/PointCloud";	//*********** TEMPORARY UNTIL WE AGREE ON WHERE IT SHOULD GO *******
		if (!WritePointCloud(filename, pointCloud, imageRectified, MESH))
			cout << endl << " ERROR in function WritePointCloud: Could not open " << filename << endl;
	}

	return 0;
}
