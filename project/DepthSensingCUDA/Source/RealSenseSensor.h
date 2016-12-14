#pragma once


#include "GlobalAppState.h"

//win8 only
#ifdef REAL_SENSE

#include "RGBDSensor.h"
#include "RealSense/pxcsensemanager.h"

#include <vector>
#include <list>

class RealSenseSensor : public RGBDSensor
{
public:

	//! Constructor; allocates CPU memory and creates handles
	RealSenseSensor();

	//! Destructor; releases allocated ressources
	virtual ~RealSenseSensor() override;

	//! Initializes the sensor
	virtual HRESULT createFirstConnected() override;

	//! Processes the depth & color data
	virtual processDepth() override;
	

	//! processing happends in processdepth()
	virtual HRESULT processColor() override{
		return S_OK;
	}

	virtual std::string getSensorName() const override{
		return "RealSense";
	}
	

protected:

	PXCSession *m_session;
	PXCCapture *m_capture;
	PXCCapture::Device *m_device;
	PXCSenseManager *m_senseManager;

	float m_depthFps;
	float m_colorFps;
};

#endif
