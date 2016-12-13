#pragma once

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"
#include "BinaryDumpReader.h"



class MultiBinaryDumpReader :
	public RGBDSensor
{
public:
	MultiBinaryDumpReader();
	virtual ~MultiBinaryDumpReader();


	//! initializes the sensor
	HRESULT createFirstConnected();

	//! reads the next depth frame
	HRESULT processDepth();

	HRESULT processColor()	{
		assert(false);	// should not call this
		return S_OK;
	}

	std::string getSensorName() const {
		return "MultiBinaryDumpSensor";
	}

	mat4f getRigidTransform(int offset) const {
		assert(false);	//should not call this
		return readers_[0].getRigidTransform(offset);
	}

	std::vector<BinaryDumpReader>& getBinaryDumpReaders(){
		return readers_;
	}

private:
	std::vector<BinaryDumpReader> readers_;
};

