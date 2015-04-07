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
#include <vector>
#include <sstream>

using namespace cv;
using namespace std;

enum ApplicationMode {CALIBRATE, RECTIFY};

// number-to-string conversion function
template <typename T> string toString(const T& t)
{
	ostringstream s;
	s << t;
	return s.str();
}

// definition of data file formats
enum FileFormat {BINARY, TEXT, MESH, MESH_TEXTURE};

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
	string rectificationImageListFile;
	string calibrationImageListFile;
	string calibrationDataDirectory;
} ;

struct PointCloud
{
	Mat data;
	float meanDistance;
	float minDistance;
	float maxDistance;
	int trimLeft;
	int trimRight;
	int trimTop;
	int trimBottom;
	float minX3D;
	float maxX3D;
	float minY3D;
	float maxY3D;
} ;

#endif