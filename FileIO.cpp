//===============================================================================//
//                                                                               //
// These functions all perform some sort of file-based I/O                       //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#include "GlobalDefines.h"
#include "FileIO.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

bool ReadImageListFromFile(string imageListFile, vector<string> &directoryList, vector<string> &imageList, bool split)
{
	// open an ascii file and read the file names as strings
	ifstream fin;
	fin.open(imageListFile);
	if(!fin.good() || fin.bad())
	{
		cout << endl << " ERROR in function ReadImageListFromFile: Could not open " << imageListFile << endl;
		return (false);
	}

	// read the full path image file names one line at a time
	string fullName;
	while (fin.good())
	{
		getline(fin, fullName);
		if (fullName == "")
			break;

		// split the full name into a directory and filename
		if (split)
		{
			unsigned found = (unsigned)fullName.find_last_of("/\\");
			directoryList.push_back(fullName.substr(0,found));
			imageList.push_back(fullName.substr(found+1));
		}
		else
			imageList.push_back(fullName);
	}

	// close the file and return
	fin.close();

	// check for empty file
	if (imageList.empty())
		return (false);
	else
		return (true);
}


bool ReadTwoImageListsFromFile(string imageListFile, vector<string> &inputList, vector<string> &outputList)
{
	// open an ascii file and read the file names as strings
	ifstream fin;
	fin.open(imageListFile);
	if(!fin.good() || fin.bad())
	{
		cout << endl << " ERROR in function ReadImageListFromFile: Could not open " << imageListFile << endl;
		return (false);
	}

	// read the full path image file names one line at a time
	string line;
	while (fin.good())
	{
		getline(fin, line);
		if (line == "")
			break;

		// check for comma delimiter and split line
		int found = (int)line.find_last_of(",");
		if (found == -1)
			continue;
		else
		{
			inputList.push_back(line.substr(0,found));
			outputList.push_back(line.substr(found+1));
		}
	}

	// close the file and return
	fin.close();

	// check for empty file
	if (inputList.empty() || outputList.empty())
		return (false);
	else
		return (true);
}


bool ReadRuntimeParameters(string filePath, Parameters &parameter)
{
	// open an ascii file for input
	ifstream fin;
	fin.open(filePath);
	if(!fin.good() || fin.bad())
		return false;

	// set default values
	parameter.doNotRectify = false;
	parameter.displayRectifiedImage = false;
	parameter.displayDisparityImage = false;
	parameter.pauseForKeystroke = false;
	parameter.nHorizontal = 0;
	parameter.nVertical = 0;
	parameter.squareSize = 0;

	// read the contents of the file a line at a time
	string line, word;
	string comment = "//";
	vector <string> wordList;
	while (getline(fin, line))
	{
		// tokenize the line into individual words
		wordList.clear();
		istringstream wordStream(line);
		while (wordStream >> word)
		{
			// check if comment or command
			if (word.substr(0,comment.length()) == comment)
				break;
			else
				wordList.push_back(word);
		}

		// parse the words
		bool haveAnotherWord;
		int nWords = (int)wordList.size();
		for (int iWord=0; iWord<nWords; iWord++)
		{
			// get this word and check if the next word is also available
			word = wordList.at(iWord);
			if (iWord+1 < nWords)
				haveAnotherWord = true;

			// commands that are followed by values
			if (word == "single_square_size" && haveAnotherWord)
				{parameter.squareSize = stof(wordList.at(++iWord)); break;}

			if (word == "horizontal_count" && haveAnotherWord)
				{parameter.nHorizontal = stoi(wordList.at(++iWord)); break;}

			if (word == "vertical_count" && haveAnotherWord)
				{parameter.nVertical = stoi(wordList.at(++iWord)); break;}

			// commands that are followed by strings
			if (word == "calibration_image_listfile")
				{parameter.calibrationImageListFile = wordList.at(++iWord); break;}

			if (word == "calibration_data_directory")
				{parameter.calibrationDataDirectory = wordList.at(++iWord); break;}

			if (word == "rectification_image_listfile")
				{parameter.rectificationImageListFile = wordList.at(++iWord); break;}

			// commands that are switches
			if (word == "do_not_rectify")
				{parameter.doNotRectify = true; break;}

			if (word == "display_rectified_image")
				{parameter.displayRectifiedImage = true; break;}

			if (word == "display_disparity_image")
				{parameter.displayDisparityImage = true; break;}

			if (word == "pause_for_keystroke")
				{parameter.pauseForKeystroke = true; break;}
		}
	}

	// close the file
	fin.close();
	return true;
}


void ValidateRuntimeParameters(Parameters parameter, ApplicationMode applicationMode)
{
	cout << "---------------------------------------------------------------------" << endl << endl;

	if (applicationMode != CALIBRATE && applicationMode != RECTIFY)
		cout << "ERROR in ValidateRuntimeParameters: applicationMode not set in argument list" << endl << endl;


	if (parameter.squareSize <= 0.0f && applicationMode == CALIBRATE)
		cout << "ERROR: command \"single_square_size\" missing or not followed by a positive value" << endl << endl;
	if (parameter.nHorizontal <= 0 && applicationMode == CALIBRATE)
		cout << "ERROR: command \"horizontal_count\" missing or not followed by a positive value" << endl << endl;
	if (parameter.nVertical <= 0 && applicationMode == CALIBRATE)
		cout << "ERROR: command \"vertical_count\" missing or not followed by a positive value" << endl << endl;


	if (parameter.rectificationImageListFile.empty() && applicationMode == RECTIFY)
		cout << "ERROR: command \"rectification_image_listfile\" missing or not followed by valid argument" << endl << endl;
	if (parameter.calibrationImageListFile.empty() && applicationMode == CALIBRATE)
		cout << "ERROR: command \"calibration_image_listfile\" missing or not followed by valid argument" << endl << endl;
	if (parameter.calibrationDataDirectory.empty())
		cout << "ERROR: command \"calibration_data_directory\" missing or not followed by valid argument" << endl << endl;

	cout << endl << "---------------------------------------------------------------------" << endl << endl << endl;
}


bool ReadIntrinsicMatrices(string calibrationDataDirectory, Mat &M1, Mat &D1, Mat &M2, Mat &D2)
{
	// get the intrinsic matrices from previous calibration
	FileStorage fs(calibrationDataDirectory + "/intrinsics.yml", CV_STORAGE_READ);
	if (fs.isOpened())
	{
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;
		fs.release();
		return true;
	}
	else
		return false;
}


bool ReadExtrinsicMatrices(string calibrationDataDirectory, Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q)
{
	// get the extrinsic matrices from previous calibration
	FileStorage fs(calibrationDataDirectory + "/extrinsics.yml", CV_STORAGE_READ);
	if( fs.isOpened() )
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
		return true;
	}
	else
		return false;
}


bool WritePointCloud(string filename, PointCloud pointCloud, Mat image, FileFormat fileFormat)
{
	if (fileFormat == TEXT)
	{
		// open an ascii file for output
		ofstream fout;
		fout.open(filename+"PointCloud.txt");
		if(!fout.good() || fout.bad())
			return false;

		// write out header indicating number of rows and columns
		Mat data = pointCloud.data;
		fout << "Rows " << data.rows << endl;
		fout << "Columns " << data.cols << endl;
		fout << "Order_By_Row" << endl;
		fout << "Mean_Distance " << (int)(pointCloud.meanDistance + 0.5f) << endl;
		fout << "Min_Distance " << (int)(pointCloud.minDistance + 0.5f) << endl;
		fout << "Max_Distance " << (int)(pointCloud.maxDistance + 0.5f) << endl;

		// write the integer values one at a time by rows
		for (int iRow=0; iRow<data.rows; iRow++)
		{
			for (int iCol=0; iCol<data.cols; iCol++)
			{
				if (data.at<Vec3f>(iRow,iCol)[2] <= 0.0f)
					fout << 0 << " " << 0 << " " << 0 << endl;
				else
					fout << (int)(data.at<Vec3f>(iRow,iCol)[0] + 0.5f) << " " << (int)(data.at<Vec3f>(iRow,iCol)[1] + 0.5f) << " " << (int)(data.at<Vec3f>(iRow,iCol)[2] + 0.5f) << endl;	// x,y,z coordinates
			}
		}

		// close the file
		fout.close();
	}
	else if (fileFormat == MESH || fileFormat == MESH_TEXTURE)
	{
		// perspective corrected output with optional texture overlay image
		float xMax = pointCloud.maxX3D;
		float xMin = pointCloud.minX3D;
		float xRange = xMax - xMin;
		float yMax = pointCloud.maxY3D;
		float yMin = pointCloud.minY3D;
		float yRange = yMax - yMin;
		float perspective;
		
		// open an ascii file for output
		ofstream fout;
		fout.open(filename+"Mesh.xyz");
		if(!fout.good() || fout.bad())
			return false;

		// write out header indicating number of rows and columns
		Mat data = pointCloud.data;
		fout << "AT3D_XYZ V01.00 Mesh A" << endl;
		fout << "Rows " << data.rows << endl;
		fout << "Columns " << data.cols << endl;
		fout << "Order By_Row" << endl;
		fout << "Data" << endl;

		// write the integer values one at a time by rows
		for (int iRow=0; iRow<data.rows; iRow++)
		{
			for (int iCol=0; iCol<data.cols; iCol++)
			{
				if (data.at<Vec3f>(iRow,iCol)[2] <= 0.0f)
				{
					// non-valid disparity  x,y,z coordinates
					fout << (int)(xMin + (xRange*(float)iCol/(float)data.cols) + 0.5f) << " ";
					fout << (int)(yMin + (yRange*(float)iRow/(float)data.rows) + 0.5f) << " ";
					fout << int(pointCloud.maxDistance + 0.5f) << endl;
				}
				else
				{
					// perspective-corrected x,y,z coordinates
					perspective = pointCloud.meanDistance / data.at<Vec3f>(iRow,iCol)[2];
					fout << (int)(data.at<Vec3f>(iRow,iCol)[0] * perspective + 0.5f) << " ";
					fout << (int)(data.at<Vec3f>(iRow,iCol)[1] * perspective + 0.5f) << " ";
					fout << (int)(data.at<Vec3f>(iRow,iCol)[2] + 0.5f) << endl;
				}
			}
		}

		// close the file
		fout.close();

		// write texture file for mapping onto point cloud mesh
		if (fileFormat == MESH_TEXTURE)
		{
			// trim and write the companion texture file
			image = image(Rect(pointCloud.trimLeft, pointCloud.trimTop,
				(image.cols/2)-pointCloud.trimLeft-pointCloud.trimRight,
				image.rows-pointCloud.trimTop-pointCloud.trimBottom));
			if (!imwrite(filename+"Texture.jpg", image))
				return false;
		}
	}
	else
	{
		// open a binary file for output
		ofstream fout;
		fout.open(filename+"PointCloud.dat", ios::binary);
		if(!fout.good() || fout.bad())
			return false;

		// write out header indicating number of rows, columns, and camera-to-subject distances
		Mat data = pointCloud.data;
		int meanDistance = (int)(pointCloud.meanDistance + 0.5f);
		int minDistance = (int)(pointCloud.minDistance + 0.5f);
		int maxDistance = (int)(pointCloud.maxDistance + 0.5f);
		fout.write((char*)&data.rows, sizeof(data.rows));
		fout.write((char*)&data.cols, sizeof(data.cols));
		fout.write((char*)&meanDistance, sizeof(meanDistance));
		fout.write((char*)&minDistance, sizeof(minDistance));
		fout.write((char*)&maxDistance, sizeof(maxDistance));

		// write the integer values one at a time by rows
		int x, y, z, zero = 0;
		for (int iRow=0; iRow<data.rows; iRow++)
		{
			for (int iCol=0; iCol<data.cols; iCol++)
			{
				if (data.at<Vec3f>(iRow,iCol)[2] <= 0.0f)
				{
					// 0, 0, 0 coordinate (invalid dispary)
					fout.write((char*)&zero, sizeof(zero));
					fout.write((char*)&zero, sizeof(zero));
					fout.write((char*)&zero, sizeof(zero));
				}
				else
				{
					// x, y, z coordinate (valid disparity)
					x = (int)( data.at<Vec3f>(iRow,iCol)[0] + 0.5f);
					y = (int)( data.at<Vec3f>(iRow,iCol)[1] + 0.5f);
					z = (int)( data.at<Vec3f>(iRow,iCol)[2] + 0.5f);
					fout.write((char*)&x, sizeof(x));
					fout.write((char*)&y, sizeof(y));
					fout.write((char*)&z, sizeof(z));
				}
			}
		}
		// close the file
		fout.close();
	}

	// done
	return true;
}

