
#include <cutil_inline.h>
#include <cutil_math.h>

#include "cuda_SimpleMatrixUtil.h"


#include "VoxelUtilHashSDF.h"
#include "DepthCameraUtil.h"

#define T_PER_BLOCK 8

struct SDFBlockDesc {
	int3 pos;
	int ptr;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Streaming from GPU to CPU: copies only selected blocks/hashEntries to the CPU if outside of the frustum //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


//-------------------------------------------------------
// Pass 1: Find all SDFBlocks that have to be transfered
//-------------------------------------------------------

__global__ void integrateFromGlobalHashPass1Kernel(VoxelHashData voxelHashData, uint start, float radius, float3 cameraPosition, uint* d_outputCounter, SDFBlockDesc* d_output) 
{
	const HashParams& hashParams = c_hashParams;
	unsigned int hashEntryIdx = blockIdx.x*blockDim.x + threadIdx.x + start;
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (hashEntryIdx < hashParams.m_hashNumBuckets*HASH_BUCKET_SIZE) {
		HashEntry& entry = voxelHashData.d_hash[hashEntryIdx];
		float3 posWorld = voxelHashData.SDFBlockToWorld(entry.pos);
		float d = length(posWorld - cameraPosition);

		if (entry.ptr != FREE_ENTRY && d >= radius) {
			// Write
			SDFBlockDesc d;
			d.pos = entry.pos;
			d.ptr = entry.ptr;

			#ifndef HANDLE_COLLISIONS
				uint addr = atomicAdd(&d_outputCounter[0], 1);
				d_output[addr] = d;
				voxelHashData.appendHeap(entry.ptr/linBlockSize);
				voxelHashData.resetHashEntry(bucketID);
			#endif
			#ifdef HANDLE_COLLISIONS
				//if there is an offset or hash doesn't belong to the bucket (linked list)
				if (entry.offset != 0 || voxelHashData.computeHashPos(entry.pos) != hashEntryIdx / HASH_BUCKET_SIZE) {					
					if (voxelHashData.deleteHashEntry(entry.pos)) {
						voxelHashData.appendHeap(d.ptr / linBlockSize);
						uint addr = atomicAdd(&d_outputCounter[0], 1);
						d_output[addr] = d;
					}
				} else {
					uint addr = atomicAdd(&d_outputCounter[0], 1);
					d_output[addr] = d;
					voxelHashData.appendHeap(d.ptr / linBlockSize);
					voxelHashData.resetHashEntry(entry);
				}
			#endif
		}
	}
}

extern "C" void integrateFromGlobalHashPass1CUDA(const HashParams& hashParams, const VoxelHashData& voxelHashData, uint threadsPerPart, uint start, float radius, const float3& cameraPosition, uint* d_outputCounter, SDFBlockDesc* d_output)
{
	const dim3 gridSize((threadsPerPart + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	if (threadsPerPart > 0) {
		// each thread will check on one hash entry
		integrateFromGlobalHashPass1Kernel<<<gridSize, blockSize>>>(voxelHashData, start, radius, cameraPosition, d_outputCounter, d_output);
	}

#ifdef _DEBUG
	cutilSafeCall(cudaDeviceSynchronize());
	cutilCheckMsg(__FUNCTION__);
#endif
}


//-------------------------------------------------------
// Pass 2: Copy SDFBlocks to output buffer
//-------------------------------------------------------


__global__ void integrateFromGlobalHashPass2Kernel(VoxelHashData voxelHashData, const SDFBlockDesc* d_SDFBlockDescs, Voxel* d_output, unsigned int nSDFBlocks)
{
	const uint idxBlock = blockIdx.x;

	if (idxBlock < nSDFBlocks) {

		const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
		const uint idxInBlock = threadIdx.x;
		const SDFBlockDesc& desc = d_SDFBlockDescs[idxBlock];

		// Copy SDF block to CPU
		d_output[idxBlock*linBlockSize + idxInBlock] = voxelHashData.d_SDFBlocks[desc.ptr + idxInBlock];

		// Reset SDF Block
		voxelHashData.resetVoxel(desc.ptr + idxInBlock);
	}
}

extern "C" void integrateFromGlobalHashPass2CUDA(const HashParams& hashParams, const VoxelHashData& voxelHashData, uint threadsPerPart, const SDFBlockDesc* d_SDFBlockDescs, Voxel* d_output, unsigned int nSDFBlocks)
{
	const uint threadsPerBlock = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	const dim3 gridSize(threadsPerPart, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (threadsPerPart > 0) {
		integrateFromGlobalHashPass2Kernel<<<gridSize, blockSize>>>(voxelHashData, d_SDFBlockDescs, d_output, nSDFBlocks);
	}

#ifdef _DEBUG
	cutilSafeCall(cudaDeviceSynchronize());
	cutilCheckMsg(__FUNCTION__);
#endif
}



///////////////////////////////////////////////////////////////////////
// Streaming from CPU to GPU: copies an entire chunk back to the GPU //
///////////////////////////////////////////////////////////////////////



//-------------------------------------------------------
// Pass 1: Allocate memory on GPU heap and insert hash entry.
//-------------------------------------------------------

__global__ void  chunkToGlobalHashPass1Kernel(VoxelHashData voxelHashData, uint numSDFBlockDescs, uint heapCountPrev, const SDFBlockDesc* d_SDFBlockDescs, const Voxel* d_SDFBlocks)
{
	const unsigned int sdfBlockIdx = blockIdx.x*blockDim.x + threadIdx.x;
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;

	if (sdfBlockIdx < numSDFBlockDescs)	{
		
		uint ptr = voxelHashData.d_heap[heapCountPrev - sdfBlockIdx] * linBlockSize;	//mass alloc

		HashEntry entry;
		entry.pos = d_SDFBlockDescs[sdfBlockIdx].pos;
		entry.offset = 0;
		entry.ptr = ptr;

		voxelHashData.insertHashEntry(entry);
	}
}

extern "C" void chunkToGlobalHashPass1CUDA(const HashParams& hashParams, const VoxelHashData& voxelHashData, uint numSDFBlockDescs, uint heapCountPrev, const SDFBlockDesc* d_SDFBlockDescs, const Voxel* d_SDFBlocks)
{
	const dim3 gridSize((numSDFBlockDescs + (T_PER_BLOCK*T_PER_BLOCK) - 1)/(T_PER_BLOCK*T_PER_BLOCK), 1);
	const dim3 blockSize((T_PER_BLOCK*T_PER_BLOCK), 1);

	if (numSDFBlockDescs > 0) {
		chunkToGlobalHashPass1Kernel<<<gridSize, blockSize>>>(voxelHashData, numSDFBlockDescs, heapCountPrev, d_SDFBlockDescs, d_SDFBlocks);
	}

#ifdef _DEBUG
	cutilSafeCall(cudaDeviceSynchronize());
	cutilCheckMsg(__FUNCTION__);
#endif
}

//-------------------------------------------------------
// Pass 2: Copy input to SDFBlocks
//-------------------------------------------------------

__global__ void chunkToGlobalHashPass2Kernel(VoxelHashData voxelHashData, uint heapCountPrev, const SDFBlockDesc* d_SDFBlockDescs, const Voxel* d_SDFBlocks)
{
	const uint blockID = blockIdx.x;
	const uint linBlockSize = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
		 
	uint ptr = voxelHashData.d_heap[heapCountPrev-blockID]*linBlockSize;

	int freeBlockIdx = voxelHashData.d_heap[heapCountPrev - blockID];
	//if (freeBlockIdx >= 262144) {
	//	printf("blocks idx: %d, heap idx: %d\n", freeBlockIdx, heapCountPrev - blockID);
	//}
	voxelHashData.d_SDFBlocks[ptr + threadIdx.x] = d_SDFBlocks[blockIdx.x*blockDim.x + threadIdx.x];
}


extern "C" void chunkToGlobalHashPass2CUDA(const HashParams& hashParams, const VoxelHashData& voxelHashData, uint numSDFBlockDescs, uint heapCountPrev, const SDFBlockDesc* d_SDFBlockDescs, const Voxel* d_SDFBlocks)
{
	const uint threadsPerBlock = SDF_BLOCK_SIZE * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE;
	const dim3 gridSize(numSDFBlockDescs, 1);
	const dim3 blockSize(threadsPerBlock, 1);

	if (numSDFBlockDescs > 0) {
		// each thread is responsible for one voxel
		chunkToGlobalHashPass2Kernel<<<gridSize, blockSize>>>(voxelHashData, heapCountPrev, d_SDFBlockDescs, d_SDFBlocks);
	}

#ifdef _DEBUG
	cutilSafeCall(cudaDeviceSynchronize());
	cutilCheckMsg(__FUNCTION__);
#endif
}