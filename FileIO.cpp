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
	fin.open(imageListFile.c_str());
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
	fin.open(imageListFile.c_str());
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
	fin.open(filePath.c_str());
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
