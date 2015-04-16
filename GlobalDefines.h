//===============================================================================//
//                                                                               //
// Header file for both stereo camera calibration and image rectification code   //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#ifndef GlobalDefines_H_
#define GlobalDefines_H_

#include <opencv2/core/core.hpp>

#include <string>
#include <sstream>

enum ApplicationMode {CALIBRATE, RECTIFY};

// number-to-string conversion function
template <typename T> std::string toString(const T& t)
{
	ostringstream s;
	s << t;
	return s.str();
}

// definition of user-controlled parameters
struct Parameters
{
	bool doNotRectify;
	bool pauseForKeystroke;
	bool displayRectifiedImage;
	bool displayDisparityImage;
	int nHorizontal;
	int nVertical;
	float squareSize;
	std::string rectificationImageListFile;
	std::string calibrationImageListFile;
	std::string calibrationDataDirectory;
} ;

#endif