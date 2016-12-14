#pragma once

#include <windows.h>
#include <cassert>
#include "Eigen.h"
#include <d3dx9math.h>

#include "mLib.h"


namespace ml {
	class SensorData;
	class RGBDFrameCacheWrite;
}

/**
 * RGBDSensor
 * A pure virtual class defining the interfaces of the sensor module.
 * All sensor module should implement these interfaces.
 */
class RGBDSensor
{
public:

	// Default constructor
	RGBDSensor();

	//! Init
	void init(unsigned int depthWidth, unsigned int depthHeight, unsigned int colorWidth, unsigned int colorHeight, unsigned int depthRingBufferSize = 1);

	//! Destructor; releases allocated ressources
	virtual ~RGBDSensor();

	//! Connected to Depth Sensor
	virtual HRESULT createFirstConnected() = 0;

	//! Process the data for next frame
	virtual HRESULT process() = 0;

	//! Returns the sensor name
	virtual std::string getSensorName() const = 0;

	//! Toggles the near-mode if available
	virtual HRESULT toggleNearMode();

	//! Get the intrinsic & extrinsic camera matrix of the sensor
	virtual const mat4f& getDepthIntrinsics() const;
	virtual const mat4f& getDepthIntrinsicsInv() const;
	virtual const mat4f& getColorIntrinsics() const;
	virtual const mat4f& getColorIntrinsicsInv() const;
	virtual const mat4f& getDepthExtrinsics() const;
	virtual const mat4f& getDepthExtrinsicsInv() const;
	virtual const mat4f& getColorExtrinsics() const;
	virtual const mat4f& getColorExtrinsicsInv() const;

	//! gets the pointer to depth array
	virtual float*			getDepthFloat();
	virtual const float*	getDepthFloat() const;

	//! gets the pointer to color array
	virtual vec4uc*			getColorRGBX();
	virtual const vec4uc*	getColorRGBX() const;

	virtual unsigned int getColorWidth() const;
	virtual unsigned int getColorHeight() const;
	virtual unsigned int getDepthWidth() const;
	virtual unsigned int getDepthHeight() const;

	virtual void reset(); 

	//! saves the point cloud of the current frame to a file
	virtual void savePointCloud(const std::string& filename, const mat4f& transform = mat4f::identity()) const;

	//! records the current frame to an internally
	virtual void recordFrame();

	//! records current transform
	virtual void recordTrajectory(const mat4f& transform);

	//! accumulates
	virtual void recordPointCloud(const mat4f& transform = mat4f::identity());
	virtual void saveRecordedPointCloud(const std::string& filename);

	//! saves all previously recorded frames to file
	virtual void saveRecordedFramesToFile(const std::string& filename);

	//! returns the current rigid transform; if not specified by the 'actual' sensor the identiy is returned
	//  Rigid Transform transforms the target camera pose to the reference camera pose.
	virtual mat4f getRigidTransform(int offset) const {
		return mat4f::identity();
	}

	virtual int getCurrentSensorIdx() const {
		return 0;
	}

	bool isCompleted() {
		return m_bCompleted;
	}


protected:

	void initializeDepthIntrinsics(float fovX, float fovY, float centerX, float centerY);
	void initializeColorIntrinsics(float fovX, float fovY, float centerX, float centerY);
	void initializeDepthExtrinsics(const Matrix3f& R, const Vector3f& t);
	void initializeColorExtrinsics(const Matrix3f& R, const Vector3f& t);
	void initializeDepthExtrinsics(const mat4f& m);
	void initializeColorExtrinsics(const mat4f& m);

	void incrementRingbufIdx();

	unsigned int m_currentRingBufIdx;

	mat4f m_depthIntrinsics;
	mat4f m_depthIntrinsicsInv;

	mat4f m_depthExtrinsics;
	mat4f m_depthExtrinsicsInv;

	mat4f m_colorIntrinsics;
	mat4f m_colorIntrinsicsInv;

	// we only store the extrinsic matrix of the reference frame (the first frame), usually
	// this is initialized as the identity matrix. The rigid transformation of the camera of 
	// other frames to the reference m_recordedTrajectory.
	mat4f m_colorExtrinsics;
	mat4f m_colorExtrinsicsInv;

	// Depth Ring Buffer
	std::vector<float*> m_depthFloat;

	// Color Buffer
	vec4uc*				m_colorRGBX;

	LONG   m_depthWidth;
	LONG   m_depthHeight;

	LONG   m_colorWidth;
	LONG   m_colorHeight;

	bool   m_bNearMode;

	// whether the capture session has completed
	bool m_bCompleted;


private:

	void computePointCurrentPointCloud(PointCloudf& pc, const mat4f& transform = mat4f::identity()) const;
	vec3f depthToSkeleton(unsigned int ux, unsigned int uy) const;
	vec3f depthToSkeleton(unsigned int ux, unsigned int uy, float depth) const;
	vec3f getNormal(unsigned int x, unsigned int y) const;

	// true if s_recordCompression
	bool m_bUseModernSensFilesForRecording;

	std::list<float*> m_recordedDepthData;
	std::list<vec4uc*>	m_recordedColorData;

	std::vector<mat4f> m_recordedTrajectory;
	std::list<PointCloudf> m_recordedPoints;

	//new recording version
	ml::SensorData* m_recordedData;
	ml::RGBDFrameCacheWrite* m_recordedDataCache;
};
