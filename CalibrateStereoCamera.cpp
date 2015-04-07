//===============================================================================//
//                                                                               //
// This funtion computes and saves the camera calibration matrices based on a    //
// list of chessboard images.                                                    //
//                                                                               //
// Input:   imageList         A vector of chessboard input images                //
//          boardSize         The number of interior corners (horiz., vert.)     //
//          squareSize        The length of edge of a single chessboard square   //
//          calibrationDataDirectory Output directory for camera matrices        //
//          displayImage      A boolean option to display images as processed    //
//          pauseForKeystroke A boolean option to pause if image is displayed    //
// Output:  None                                                                 //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                            (based on public example from OpenCV site)         //
//                                                                               //
//===============================================================================//

/* Given a list of chessboard images, the number of corners (nx, ny)	*/
/* on the chessboards, calibrate the cameras and display the			*/
/* rectified results along with the computed disparity images.			*/

#include "CalibrateStereoCamera.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

void CalibrateStereoCamera(const vector<string>& imageList, Size boardSize, float squareSize, string calibrationDataDirectory,
							bool displayImage, bool pauseForKeystroke)
{
	int i, j, k, nImages;
	const int maxScale = 2;
	vector<string> goodImageList;
	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	// allocate storage
	nImages = (int)(imageList.size())/2;
	imagePoints[0].resize(nImages);
	imagePoints[1].resize(nImages);

	// check if image list has pairs of images
	if( imageList.size() % 2 != 0 )
	{
		cout << "Error: the image list contains odd (non-even) number of elements" << endl;
		return;
	}

	//------------------------------------------------------------------------------------------------------------------
	// find chessboard corners in image pairs
	//------------------------------------------------------------------------------------------------------------------

	// process the calibration images
	for (i=j=0; i<nImages; i++)
	{
		// process pairs of images
		for (k=0; k<2; k++)
		{
			const string& filename = imageList[i*2+k];
			Mat image = imread(filename, 0);
			if (image.empty())
				break;
			if (imageSize == Size())
				imageSize = image.size();
			else if (image.size() != imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair" << endl;
				break;
			}

			// find the chessboard inner corners
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale=1; scale<=maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = image;
				else
					resize(image, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if( found )
				{
					if( scale > 1 )
					{
						Mat cornersMat(corners);
						cornersMat *= 1./scale;
					}
					break;
				}
			}
			// display the chessboard with inner corners overlaid
			if (displayImage)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(image, cimg, CV_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640./MAX(image.rows, image.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if( c == 27 || c == 'q' || c == 'Q' )	// allow ESC or "q" to quit
					exit(-1);

            }
			else
				putchar('.');

			// no corners were found so skip to next image pair
			if (!found)
				break;

			// found corners, so refine them to sub-pixel accuracy
			cornerSubPix(image, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
		}

		// found corners on a pair of images, so add them to the "good" list
		if (k == 2)
		{
			goodImageList.push_back(imageList[i*2]);
			goodImageList.push_back(imageList[i*2+1]);
			j++;
		}
	}

	// report on success
	cout << j << " image pairs have been successfully detected." << endl;
	if( j < 2 )
	{
		cout << "Error: too few pairs of images to run the calibration" << endl;
		return;
	}

	//------------------------------------------------------------------------------------------------------------------
	// compute camera calibration (intrinsic) matrices based on the chessboard corners that were found
	//------------------------------------------------------------------------------------------------------------------

	// allocate storage again using the good image count
	nImages = j;
	imagePoints[0].resize(nImages);
	imagePoints[1].resize(nImages);
	objectPoints.resize(nImages);
    
	// generate world coordinates object points (in mm) of calibration chessboard pattern
	for (i=0; i<nImages; i++)
	{
		for (j=0; j<boardSize.height; j++)
			for (k=0; k<boardSize.width; k++)
				objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
	}
    
	// generate identity matrices
	Mat cameraMatrix[2], distortionCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;

    // perform the stereo calibration
	cout << "Running stereo calibration...";
	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
					cameraMatrix[0], distortionCoeffs[0],
					cameraMatrix[1], distortionCoeffs[1],
					imageSize, R, T, E, F,
					TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					CV_CALIB_FIX_ASPECT_RATIO +
					CV_CALIB_ZERO_TANGENT_DIST +
					CV_CALIB_SAME_FOCAL_LENGTH);
	cout << " Done" << endl << "RMS error = " << rms << endl;

	//------------------------------------------------------------------------------------------------------------------
	// validate the camera calibration (intrinsic) matrices and save
	//------------------------------------------------------------------------------------------------------------------

	// check the quality of calibration using the epipolar geometry constraint: m2^t*F*m1=0
	int nTotalPoints = 0;
	double err = 0;
	vector<Vec3f> lines[2];
	for( i = 0; i < nImages; i++ )
	{
		int nPoints = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k=0; k<2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distortionCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
		}
		for (j=0; j<nPoints; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
								imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
							fabs(imagePoints[1][i][j].x*lines[0][j][0] +
								imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		nTotalPoints += nPoints;
	}
	cout << "Average reprojection error = " <<  err/nTotalPoints << endl;
	
	// save intrinsic parameters: M camera matrix, D distortion coefficients (needed for rectification)
	FileStorage fs(calibrationDataDirectory + "/intrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distortionCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distortionCoeffs[1];
		fs.release();
	}
	else
		cout << "Error in CalibrateStereoCamera: Can't save the intrinsic parameters" << endl;
	
	//------------------------------------------------------------------------------------------------------------------
	// rectify images using camera calibration matrices and compute extrinsic matrices
	//------------------------------------------------------------------------------------------------------------------

	// compute the extrinsic parameters
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	stereoRectify(cameraMatrix[0], distortionCoeffs[0],
				  cameraMatrix[1], distortionCoeffs[1],
				  imageSize, R, T, R1, R2, P1, P2, Q,
				  CALIB_ZERO_DISPARITY, -1.0, imageSize, &validRoi[0], &validRoi[1]);

	// Save the extrinsic parameters
	fs.open(calibrationDataDirectory + "/extrinsics.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error in CalibrateStereoCamera: Can't save the intrinsic parameters" << endl;

	//------------------------------------------------------------------------------------------------------------------
	// display rectified images
	//------------------------------------------------------------------------------------------------------------------

	// display rectification
	if (!displayImage)
		return;

	// Precompute maps for cv::remap() used to create undistorted rectified images
	Mat rmap[2][2];
	initUndistortRectifyMap(cameraMatrix[0], distortionCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distortionCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
	if( !isVerticalStereo )
	{
		sf = 0.5;		// 600./MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w*2, CV_8UC3);
	}
	else
	{
		sf = 0.5;		// 300./MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h*2, w, CV_8UC3);
	}

	for( i = 0; i < nImages; i++ )
	{
		for( k = 0; k < 2; k++ )
		{
			Mat rimg, cimg;
			Mat image = imread(goodImageList[i*2+k], 0);
			remap(image, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
			cvtColor(rimg, cimg, CV_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
		}

		// draw epipolar lines
		if (!isVerticalStereo)
			for (j =0; j<canvas.rows; j+=16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j=0; j<canvas.cols; j+=16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);

		// display the rectified images with overlays
		imshow("rectified", canvas);
		if (pauseForKeystroke)
		{
			char c = (char)waitKey();
			if( c == 27 || c == 'q' || c == 'Q' )	// allow ESC or "q" to quit
				exit(-1);
		}
	}
}
