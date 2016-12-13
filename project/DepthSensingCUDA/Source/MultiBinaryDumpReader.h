#pragma once

#ifdef BINARY_DUMP_READER

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"
#include "BinaryDumpReader.h"

/* XXX
Ideally I should use RGBDSensor as base class.
I use the first file as a surrogate to service some base class methods
to avoid breaking the program everywhere.
This will consumes extra resource.
*/

class MultiBinaryDumpReader :
	public BinaryDumpReader
{
public:
	MultiBinaryDumpReader();
	virtual ~MultiBinaryDumpReader();


	//! initializes the sensor
	HRESULT createFirstConnected();

	//! reads the next depth frame
	HRESULT processDepth();

	HRESULT processColor()	{
		//everything done in process depth since order is relevant (color must be read first)
		return S_OK;
	}

	std::string getSensorName() const {
		// XXX
		return BinaryDumpReader::getSensorName();
	}

	mat4f getRigidTransform(int offset) const {
		assert(false);	//should not call this
		return BinaryDumpReader::getRigidTransform(offset);
	}

	std::vector<BinaryDumpReader>& getBinaryDumpReaders(){
		return readers_;
	}

private:
	std::vector<BinaryDumpReader> readers_;
};


#endif
