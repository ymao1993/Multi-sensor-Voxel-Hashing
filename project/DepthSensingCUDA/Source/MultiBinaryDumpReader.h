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
		// XXX
		return BinaryDumpReader::getSensorName();
	}

	mat4f getRigidTransform(int offset) const {
		// XXX
		return BinaryDumpReader::getRigidTransform(offset);
	}

	mat4f getRigidTransformById(size_t id, int offset) const {
		return readers_[id].getRigidTransform(offset);
	}

	size_t getActiveId() const {
		return active_id_;
	}

	void setActiveId(size_t id){
		active_id_ = id;
	}


private:
	std::vector<BinaryDumpReader> readers_;
	size_t active_id_ = 0;
};

