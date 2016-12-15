#include "stdafx.h"

#include "CUDARGBDAdapter.h"
#include "TimingLog.h"

extern "C" void copyFloat4Map(float4* d_output, float4* d_input, unsigned int width, unsigned int height);

extern "C" void convertDepthRawToFloat(float* d_output, unsigned short* d_input, unsigned int width, unsigned int height, float minDepth, float maxDepth);
extern "C" void convertColorRawToFloat4(float4* d_output, BYTE* d_input, unsigned int width, unsigned int height);

extern "C" void resampleFloatMap(float* d_colorMapResampledFloat, unsigned int outputWidth, unsigned int outputHeight, float* d_colorMapFloat, unsigned int inputWidth, unsigned int inputHeight, float* d_depthMaskMap);
extern "C" void resampleFloat4Map(float4* d_colorMapResampledFloat4, unsigned int outputWidth, unsigned int outputHeight, float4* d_colorMapFloat4, unsigned int inputWidth, unsigned int inputHeight);


#define BUFFER_SIZE 5

CUDARGBDAdapter::CUDARGBDAdapter()
{
	// depth
	d_depthMapFloat = NULL;
	d_depthMapResampledFloat = NULL;

	// color
	d_colorMapRaw = NULL;
	d_colorMapFloat4 = NULL;
	d_colorMapResampledFloat4 = NULL;

	m_frameNumber = 0;
}

CUDARGBDAdapter::~CUDARGBDAdapter()
{
}

void CUDARGBDAdapter::OnD3D11DestroyDevice() 
{
	cutilSafeCall(cudaFree(d_depthMapFloat));
	cutilSafeCall(cudaFree(d_depthMapResampledFloat));
	cutilSafeCall(cudaFree(d_colorMapRaw));
	cutilSafeCall(cudaFree(d_colorMapFloat4));
	cutilSafeCall(cudaFree(d_colorMapResampledFloat4));
}

void CUDARGBDAdapter::allocateGPUBuffers(BYTE** pp_colorMapRaw, 
										 float4** pp_colorMapFloat4,
										 float4** pp_colorMapResampledFloat4,
										 float** pp_depthMapFloat,
										 float** pp_depthMapResampledFloat)
{
	// depth input buffering on GPU
	const unsigned int bufferDimDepthInput = m_RGBDSensor->getDepthWidth()*m_RGBDSensor->getDepthHeight();
	cutilSafeCall(cudaMalloc(pp_depthMapFloat, sizeof(float) * bufferDimDepthInput));

	// color input buffering on GPU
	const unsigned int bufferDimColorInput = m_RGBDSensor->getColorWidth()*m_RGBDSensor->getColorHeight();
	cutilSafeCall(cudaMalloc(pp_colorMapRaw, sizeof(unsigned int) * bufferDimColorInput));
	cutilSafeCall(cudaMalloc(pp_colorMapFloat4, 4 * sizeof(float) * bufferDimColorInput));

	// depth&color output buffering on GPU
	const unsigned int bufferDimOutput = m_width*m_height;
	cutilSafeCall(cudaMalloc(pp_depthMapResampledFloat, sizeof(float) * bufferDimOutput));
	cutilSafeCall(cudaMalloc(pp_colorMapResampledFloat4, 4 * sizeof(float) * bufferDimOutput));
}

void CUDARGBDAdapter::updateCameraMatrices()
{
	const unsigned int widthColor = m_RGBDSensor->getColorWidth();
	const unsigned int heightColor = m_RGBDSensor->getColorHeight();
	const unsigned int widthDepth = m_RGBDSensor->getDepthWidth();
	const unsigned int heightDepth = m_RGBDSensor->getDepthHeight();

	// adapt depth intrinsics if needed
	m_depthIntrinsics = m_RGBDSensor->getDepthIntrinsics();
	m_depthIntrinsics._m00 *= (float)m_width / (float)widthDepth;				//focal length
	m_depthIntrinsics._m11 *= (float)m_height / (float)heightDepth;			//focal length
	m_depthIntrinsics._m02 *= (float)(m_width - 1) / (float)(widthDepth - 1);		//principal point
	m_depthIntrinsics._m12 *= (float)(m_height - 1) / (float)(heightDepth - 1);	//principal point
	m_depthIntrinsicsInv = m_depthIntrinsics.getInverse();

	// adapt color intrinsics if needed
	m_colorIntrinsics = m_RGBDSensor->getColorIntrinsics();
	m_colorIntrinsics._m00 *= (float)m_width / (float)widthColor;				//focal length
	m_colorIntrinsics._m11 *= (float)m_height / (float)heightColor;			//focal length
	m_colorIntrinsics._m02 *= (float)(m_width - 1) / (float)(widthColor - 1);		//principal point
	m_colorIntrinsics._m12 *= (float)(m_height - 1) / (float)(heightColor - 1);	//principal point
	m_colorIntrinsicsInv = m_colorIntrinsics.getInverse();

	// adapt depth&color extrinsics
	m_depthExtrinsics = m_RGBDSensor->getDepthExtrinsics();
	m_depthExtrinsicsInv = m_RGBDSensor->getDepthExtrinsicsInv();
	m_colorExtrinsics = m_RGBDSensor->getColorExtrinsics();
	m_colorExtrinsicsInv = m_RGBDSensor->getColorExtrinsicsInv();
}


HRESULT CUDARGBDAdapter::OnD3D11CreateDevice(ID3D11Device* device, RGBDSensor* RGBDSensor, unsigned int width, unsigned int height)
{
	m_RGBDSensor = RGBDSensor;
	m_width = width;
	m_height = height;
	updateCameraMatrices();
	allocateGPUBuffers(&d_colorMapRaw, &d_colorMapFloat4, &d_colorMapResampledFloat4, &d_depthMapFloat, &d_depthMapResampledFloat);
	return S_OK;
}

void CUDARGBDAdapter::resampleDepthColor(BYTE* colorMapRaw, 
										float4* colorMapFloat4, 
										float4* colorMapResampledFloat4, 
										float* d_depthMapFloat, 
										float* depthMapResampledFloat)
{
	// process depth
	const unsigned int bufferDimColorInput = m_RGBDSensor->getColorWidth()*m_RGBDSensor->getColorHeight();
	cutilSafeCall(cudaMemcpy(colorMapRaw, m_RGBDSensor->getColorRGBX(), sizeof(unsigned int)*bufferDimColorInput, cudaMemcpyHostToDevice));
	convertColorRawToFloat4(colorMapFloat4, colorMapRaw, m_RGBDSensor->getColorWidth(), m_RGBDSensor->getColorHeight());
	if ((m_RGBDSensor->getColorWidth() == m_width) && (m_RGBDSensor->getColorHeight() == m_height))
		copyFloat4Map(colorMapResampledFloat4, colorMapFloat4, m_width, m_height);
	else 
		resampleFloat4Map(colorMapResampledFloat4, m_width, m_height, colorMapFloat4, m_RGBDSensor->getColorWidth(), m_RGBDSensor->getColorHeight());

	// process color
	const unsigned int bufferDimDepthInput = m_RGBDSensor->getDepthWidth()*m_RGBDSensor->getDepthHeight();
	cutilSafeCall(cudaMemcpy(d_depthMapFloat, m_RGBDSensor->getDepthFloat(), sizeof(float)*bufferDimDepthInput, cudaMemcpyHostToDevice));
	resampleFloatMap(depthMapResampledFloat, m_width, m_height, d_depthMapFloat, m_RGBDSensor->getDepthWidth(), m_RGBDSensor->getDepthHeight(), NULL);

}

HRESULT CUDARGBDAdapter::process(ID3D11DeviceContext* context)
{
	if (m_RGBDSensor->process() != S_OK)	return S_FALSE;
	resampleDepthColor(d_colorMapRaw, d_colorMapFloat4, d_colorMapResampledFloat4, d_depthMapFloat, d_depthMapResampledFloat);
	m_frameNumber++;
	return S_OK;
}

