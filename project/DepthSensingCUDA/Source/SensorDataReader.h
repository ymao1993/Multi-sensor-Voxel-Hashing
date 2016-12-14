#pragma once


/************************************************************************/
/* Reads sensor data files from .sens files                            */
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"

#ifdef SENSOR_DATA_READER

namespace ml {
	class SensorData;
	class RGBDFrameCacheRead;
}

class SensorDataReader : public RGBDSensor
{
public:

	//! Constructor
	SensorDataReader();

	//! Destructor; releases allocated ressources
	virtual ~SensorDataReader() override;

	//! initializes the sensor
	virtual HRESULT createFirstConnected() override;

	//! reads the next depth frame
	virtual HRESULT process() override;

	virtual std::string getSensorName() const override;

	virtual mat4f getRigidTransform(int offset) const override;


	const SensorData* getSensorData() const {
		return m_sensorData;
	}
private:
	//! deletes all allocated data
	void releaseData();

	ml::SensorData* m_sensorData;
	ml::RGBDFrameCacheRead* m_sensorDataCache;

	unsigned int	m_numFrames;
	unsigned int	m_currFrame;
	bool			m_bHasColorData;

};


#endif	//sensor data reader
