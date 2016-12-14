#pragma once

#ifndef MULTISENSOR_H
#define MULTISENSOR_H

/************************************************************************/
/*     A composite virtual sensor that reads from multiple sources      */
/************************************************************************/

#include "GlobalAppState.h"
#include "RGBDSensor.h"
#include "stdafx.h"

#ifdef MULTI_SENSOR

class MultiSensor : public RGBDSensor
{
public:

	//! Constructor
	MultiSensor::MultiSensor(const vector<RGBDSensor*>& sensors);

	//! Destructor; releases allocated ressources
	~MultiSensor();

	//! initializes the sensor
	virtual HRESULT createFirstConnected();

	//! reads the next depth frame
	virtual HRESULT process();

	virtual std::string getSensorName() const {
		return "Multiple Sensors";
	}

	virtual mat4f getRigidTransform(int offset) const override;

	virtual unsigned int getColorWidth() const override;
	virtual unsigned int getColorHeight() const override;
	virtual unsigned int getDepthWidth() const override;
	virtual unsigned int getDepthHeight() const override;
	virtual const mat4f& getDepthIntrinsics() const override;
	virtual const mat4f& getDepthIntrinsicsInv() const override;
	virtual const mat4f& getColorIntrinsics() const override;
	virtual const mat4f& getColorIntrinsicsInv() const override;
	virtual const mat4f& getDepthExtrinsics() const override;
	virtual const mat4f& getDepthExtrinsicsInv() const override;
	virtual const mat4f& getColorExtrinsics() const override;
	virtual const mat4f& getColorExtrinsicsInv() const override;

	//! gets the pointer to depth array
	virtual float* getDepthFloat() override;
	virtual const float* getDepthFloat() const override;

	//! gets the pointer to color array
	virtual vec4uc* getColorRGBX() override;
	virtual const vec4uc* getColorRGBX() const override;

	virtual void reset() override;

	virtual void recordFrame() override {}
	virtual void recordTrajectory(const mat4f& transform) override {}
	virtual void recordPointCloud(const mat4f& transform) override {}
	virtual void savePointCloud(const std::string& filename, const mat4f& transform = mat4f::identity()) const override {
		std::cout << "savePointCloud is not supported by MultiSensor" << std::endl;
	}
	virtual void saveRecordedPointCloud(const std::string& filename) override {
		std::cout << "saveRecordedPointCloud is not supported by MultiSensor" << std::endl;
	}
	virtual void saveRecordedFramesToFile(const std::string& filename) override {
		std::cout << "saveRecordedFramesToFile is not supported by MultiSensor" << std::endl;
	}

	RGBDSensor* getCurSensor() {
		return sensors[curSensorIdx];
	}

	int getSensorNum() {
		return (int)sensors.size();
	}


private:
	bool hasNextFrame();
	bool switchSensorIdx();
	std::vector<RGBDSensor*> sensors;
	int curSensorIdx = 0;
};

#endif
#endif