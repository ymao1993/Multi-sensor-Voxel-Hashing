#include "stdafx.h"
#include "MultiSensor.h"

#ifdef MULTI_SENSOR

MultiSensor::MultiSensor(const vector<RGBDSensor*>& sensors) {
	this->sensors = sensors;
	if (this->sensors.size() == 0) {
		throw MLIB_EXCEPTION("sensors provided to MultiSensor constructor cannot be empty");
	}
	switchSensorIdx();
}

MultiSensor::~MultiSensor() {
	for (RGBDSensor *sensor : sensors) {
		delete sensor;
	}
	sensors.clear();
}

HRESULT MultiSensor::createFirstConnected()
{
	for (RGBDSensor* sensor : sensors) {
		sensor->createFirstConnected();
	}
	return S_OK;
}

//! reads the next depth frame
HRESULT MultiSensor::process()
{
	if (!m_bCompleted)
	{
		if (switchSensorIdx()) {
			return sensors[curSensorIdx]->process();
		}
		else {
			m_bCompleted = true;
		}
	}
	return S_FALSE;
}

// ! randomly switch sensor_idx to the next available sensor
// return false if none of the sensors are available
bool MultiSensor::switchSensorIdx()
{
	vector<int> runnigSensors;
	for (int i = 0; i < sensors.size(); i++) {
		if (!sensors[i]->isCompleted()) {
			runnigSensors.push_back(i);
		}
	}
	if (runnigSensors.size() == 0) {
		curSensorIdx = -1;
		return false;
	}
	else {
		int randIdx = rand() % runnigSensors.size();
		curSensorIdx = runnigSensors[randIdx];
		return true;
	}
}

bool MultiSensor::hasNextFrame()
{
	for (int i = 0; i < sensors.size(); i++) {
		if (!sensors[i]->isCompleted()) {
			return true;
		}
	}
	return false;
}

mat4f MultiSensor::getRigidTransform(int offset) const {
	return sensors[curSensorIdx]->getRigidTransform(offset);
}

unsigned int MultiSensor::getColorWidth() const {
	return sensors[curSensorIdx]->getColorWidth();
}

unsigned int MultiSensor::getColorHeight() const {
	return sensors[curSensorIdx]->getColorHeight();
}

unsigned int MultiSensor::getDepthWidth() const {
	return sensors[curSensorIdx]->getDepthWidth();
}

unsigned int MultiSensor::getDepthHeight() const {
	return sensors[curSensorIdx]->getDepthHeight();
}

const mat4f& MultiSensor::getDepthIntrinsics() const  {
	return sensors[curSensorIdx]->getDepthIntrinsics();
}

const mat4f& MultiSensor::getDepthIntrinsicsInv() const  {
	return sensors[curSensorIdx]->getDepthIntrinsicsInv();
}

const mat4f& MultiSensor::getColorIntrinsics() const   {
	return sensors[curSensorIdx]->getColorIntrinsics();
}

const mat4f& MultiSensor::getColorIntrinsicsInv() const  {
	return sensors[curSensorIdx]->getColorIntrinsicsInv();
}

const mat4f& MultiSensor::getDepthExtrinsics() const  {
	return sensors[curSensorIdx]->getDepthExtrinsics();
}

const mat4f& MultiSensor::getDepthExtrinsicsInv() const  {
	return sensors[curSensorIdx]->getDepthExtrinsicsInv();
}

const mat4f& MultiSensor::getColorExtrinsics() const  {
	return sensors[curSensorIdx]->getColorExtrinsics();
}

const mat4f& MultiSensor::getColorExtrinsicsInv() const  {
	return sensors[curSensorIdx]->getColorExtrinsicsInv();
}

//! gets the pointer to depth array
float* MultiSensor::getDepthFloat()  {
	return sensors[curSensorIdx]->getDepthFloat();
}

const float* MultiSensor::getDepthFloat() const  {
	return sensors[curSensorIdx]->getDepthFloat();
}

//! gets the pointer to color array
vec4uc* MultiSensor::getColorRGBX()  {
	return sensors[curSensorIdx]->getColorRGBX();
}

const vec4uc* MultiSensor::getColorRGBX() const  {
	return sensors[curSensorIdx]->getColorRGBX();
}

void MultiSensor::reset() {
	for (RGBDSensor* sensor : sensors) {
		sensor->reset();
	}
}

#endif