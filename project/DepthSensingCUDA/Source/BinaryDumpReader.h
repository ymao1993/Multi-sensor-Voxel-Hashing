#pragma once


/************************************************************************/
/* Reads binary dump data from .sensor files                            */
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"

#ifdef BINARY_DUMP_READER

class BinaryDumpReader : public RGBDSensor
{
public:

	//! Constructor
	BinaryDumpReader(const std::string& filename);

	//! Destructor; releases allocated ressources
	virtual ~BinaryDumpReader() override;

	//! initializes the sensor
	virtual HRESULT createFirstConnected() override;

	//! reads the next depth frame
	virtual HRESULT process() override;

	virtual std::string getSensorName() const override{
		return m_data.m_SensorName;
	}

	virtual mat4f getRigidTransform(int offset) const override{
		unsigned int idx = m_CurrFrame + offset;
		if (idx >= m_data.m_trajectory.size()) {
			throw MLIB_EXCEPTION("invalid trajectory index " + std::to_string(idx));
		}
		return m_data.m_trajectory[idx];
	}

private:
	//! deletes all allocated data
	void releaseData();

	CalibratedSensorData m_data;

	int	m_NumFrames;
	int	m_CurrFrame;
	bool			m_bHasColorData;

	std::string filename;
};


#endif
