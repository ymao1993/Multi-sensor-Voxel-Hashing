
#include "stdafx.h"

#include "BinaryDumpReader.h"
#include "GlobalAppState.h"
#include "MatrixConversion.h"

#ifdef BINARY_DUMP_READER

#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <string>

#include <conio.h>

BinaryDumpReader::BinaryDumpReader(const std::string& filename)
{
	m_NumFrames = 0;
	m_CurrFrame = 0;
	m_bHasColorData = false;
	this->filename = filename;
}

BinaryDumpReader::~BinaryDumpReader()
{
	releaseData();
}

/**
 * createFirstConnected
 * Load the binary data from disk, this includes color data, depth data and calibration matrices.
 */
HRESULT BinaryDumpReader::createFirstConnected()
{
	releaseData();

	std::cout << "Start loading binary dump" << std::endl;
	//BinaryDataStreamZLibFile inputStream(filename, false);
	BinaryDataStreamFile inputStream(filename, false);
	inputStream >> m_data;
	std::cout << "Loading finished" << std::endl;
	std::cout << m_data << std::endl;

	std::cout << "intrinsics:" << std::endl;
	std::cout << m_data.m_CalibrationDepth.m_Intrinsic << std::endl;

	RGBDSensor::init(m_data.m_DepthImageWidth, m_data.m_DepthImageHeight, std::max(m_data.m_ColorImageWidth,1u), std::max(m_data.m_ColorImageHeight,1u), 1);
	initializeDepthIntrinsics(m_data.m_CalibrationDepth.m_Intrinsic(0,0), m_data.m_CalibrationDepth.m_Intrinsic(1,1), m_data.m_CalibrationDepth.m_Intrinsic(0,2), m_data.m_CalibrationDepth.m_Intrinsic(1,2));
	initializeColorIntrinsics(m_data.m_CalibrationColor.m_Intrinsic(0,0), m_data.m_CalibrationColor.m_Intrinsic(1,1), m_data.m_CalibrationColor.m_Intrinsic(0,2), m_data.m_CalibrationColor.m_Intrinsic(1,2));

	initializeDepthExtrinsics(m_data.m_CalibrationDepth.m_Extrinsic);
	initializeColorExtrinsics(m_data.m_CalibrationColor.m_Extrinsic);


	m_NumFrames = m_data.m_DepthNumFrames;
	assert(m_data.m_ColorNumFrames == m_data.m_DepthNumFrames || m_data.m_ColorNumFrames == 0);		
		
	if (m_data.m_ColorImages.size() > 0) {
		m_bHasColorData = true;
	} else {
		m_bHasColorData = false;
	}
	return S_OK;
}

/**
 * process
 * read the next frame's depth and color data and store them into depth ring buffer and the current color buffer. 
 */
HRESULT BinaryDumpReader::process()
{
	if (m_bCompleted) return S_FALSE;
	m_CurrFrame++;
	if(m_CurrFrame >= m_NumFrames)
	{
		m_bCompleted = true;
		std::cout << "binary dump sequence '" << filename <<"' completed" << std::endl;
		m_CurrFrame = -1;
	}

	if (!m_bCompleted) {
		float* depth = getDepthFloat();
		memcpy(depth, m_data.m_DepthImages[m_CurrFrame], sizeof(float)*getDepthWidth()*getDepthHeight());
		incrementRingbufIdx();
		if (m_bHasColorData) {
			memcpy(m_colorRGBX, m_data.m_ColorImages[m_CurrFrame], sizeof(vec4uc)*getColorWidth()*getColorHeight());
		}
		return S_OK;
	} else {
		return S_FALSE;
	}
}

void BinaryDumpReader::releaseData()
{
	m_CurrFrame = -1;
	m_bHasColorData = false;
	m_data.deleteData();
}



#endif
