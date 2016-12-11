#include "stdafx.h"
#include "MultiBinaryDumpReader.h"
#include "GlobalAppState.h"
#include "MatrixConversion.h"

#include <sstream>

MultiBinaryDumpReader::MultiBinaryDumpReader()
{
}


MultiBinaryDumpReader::~MultiBinaryDumpReader()
{
	// They will release themselves properly
}



/**
* createFirstConnected
* Load the binary data from disk, this includes color data, depth data and calibration matrices.
*/
HRESULT MultiBinaryDumpReader::createFirstConnected()
{
	

	// TODO Obtain file names from parameter
	std::string s_filelist = GlobalAppState::get().s_binaryDumpSensorFileList;
	std::istringstream ss(s_filelist);
	std::string token;
	std::vector<std::string> filelist;
	while (std::getline(ss, token, ',')) {
		filelist.push_back(token);
	}

	std::cout << "[15769] Loading multiple binary dump files:" << std::endl;
	for (auto s : filelist){
		std::cout << s << std::endl;
	}

	for (auto filename : filelist){
		readers_.emplace_back();
		readers_.back().createFirstConnected(filename);
	}

	return S_OK;
}

/**
* processDepth()
* read the next frame's depth and color data and store them into depth ring buffer and the current color buffer.
*/
HRESULT MultiBinaryDumpReader::processDepth()
{
	for (auto& r : readers_){
		// TODO Not very clear what's done here.
		r.processDepth();
	}

	return S_OK;
}
