#pragma once

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
		return readers_[0].getSensorName();
	}

	mat4f getRigidTransform(int offset) const {
		return readers_[0].getRigidTransform(offset);
	}

	mat4f getRigidTransformById(size_t id, int offset) const {
		return readers_[id].getRigidTransform(offset);
	}


private:
	std::vector<BinaryDumpReader> readers_;
};

