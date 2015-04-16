//===============================================================================//
//                                                                               //
// Header for FileIO.cpp                                                         //
//                                                                               //
// Author:                    Peter Honig, phonig@whoi.edu, March 1 2015         //
//                                                                               //
//===============================================================================//

#ifndef FileIO_H_
#define FileIO_H_

#include "GlobalDefines.h"
#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

bool ReadImageListFromFile(std::string imageListFile, std::vector<std::string> &directoryList, std::vector<std::string> &imageList, bool split=true);
bool ReadTwoImageListsFromFile(std::string imageListFile, std::vector<std::string> &inputList, std::vector<std::string> &outputList);
bool ReadRuntimeParameters(std::string filePath, Parameters &parameter);
void ValidateRuntimeParameters(Parameters parameter, ApplicationMode applicationMode);

#endif