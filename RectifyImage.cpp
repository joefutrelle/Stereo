//===============================================================================//
//                                                                               //
// This function rectifies a joined pair of stereo images given a set of camera  //
// calibration matrices and then returns the rectified joined image pair.        //
//                                                                               //
// Input:   image             Input image (joined left and right)                //
//          M1, D1, etc.      Camera calibration matrices                        //
//          displayImage      A boolean option to display images as processed    //
//          pauseForKeystroke A boolean option to pause if image is displayed    //
// Output:  returned value    A rectified joined image pair                      //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                            (based on an example from the OpenCV site)         //
//                                                                               //
//===============================================================================//

#include "GlobalDefines.h"
#include "RectifyImage.h"
#include "FileIO.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Mat RectifyImage(Mat image, Mat M1, Mat D1, Mat R1, Mat P1, Mat M2, Mat D2, Mat R2, Mat P2, bool displayImage, bool pauseForKeystroke)
{
	Mat imageLeft, imageRight, imageLeftRectified, imageRightRectified, imageRectified;

	// split image pair into two separate images
	imageLeft = image(Rect(0, 0, image.cols/2, image.rows)).clone();
	imageRight = image(Rect(image.cols/2, 0, image.cols/2, image.rows)).clone();

	// precompute maps used to create undistorted rectified images
	Size imageSize = imageLeft.size();
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, imageSize, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, imageSize, CV_16SC2, map21, map22);

	// remap the original images into rectification space
	remap(imageLeft, imageLeftRectified, map11, map12, CV_INTER_LINEAR);
	remap(imageRight, imageRightRectified, map21, map22, CV_INTER_LINEAR);

	// combine left and right rectified images
	hconcat (imageLeftRectified, imageRightRectified, imageRectified);

	// display the rectified images
	if (displayImage)
	{
		// create a display canvas
		Mat canvas, imageTemp;
		int w, h;
		double sf = 0.5;	// scale factor for display
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w*2, CV_8UC3);

		imageTemp = imageLeftRectified.clone();
		Mat canvasPart = canvas(Rect(w*0, 0, w, h));
		resize(imageTemp, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

		imageTemp = imageRightRectified.clone();
		canvasPart = canvas(Rect(w*1, 0, w, h));
		resize(imageTemp, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

		// draw epipolar lines
		for (int j=0; j<canvas.rows; j+=16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

		// display the rectified images with overlays
		imshow("rectified", canvas);
		if (pauseForKeystroke)
		{
			char c = (char)waitKey();
			if( c == 27 || c == 'q' || c == 'Q' )	// allow for ESC or "q" to quit
				exit (-1);
		}
	}

	return (imageRectified);
}
