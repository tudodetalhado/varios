/*
* Copyright 1993-2015 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*
*/

// Simple 3D volume renderer

#ifndef _VOLUMERENDER_KERNEL_CU_
#define _VOLUMERENDER_KERNEL_CU_

#include "CudaKernels.h"
#include <helper_cuda.h>
#include <helper_math.h>

typedef unsigned int  uint;
typedef unsigned char uchar;

cudaArray *d_volumeArray = 0;
cudaArray *d_transferFuncArray;

//typedef unsigned char VolumeType;
typedef float2 VolumeType;

//texture<VolumeType, 3, cudaReadModeNormalizedFloat> tex;         // 3D texture
texture<VolumeType, 3, cudaReadModeElementType>		volumeTex;         // 3D texture
texture<float4, 1, cudaReadModeElementType>         volumeTransferTex; // 1D transfer function texture

typedef struct
{
	float4 m[3];
} float3x4;

__constant__ float3x4 c_invViewMatrix;  // inverse view matrix

struct Ray
{
	float3 o;   // origin
	float3 d;   // direction
};

// intersect ray with a box
// http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm

__device__
int intersectBox(Ray r, float3 boxmin, float3 boxmax, float *tnear, float *tfar)
{
	// compute intersection of ray with all six bbox planes
	float3 invR = make_float3(1.0f) / r.d;
	float3 tbot = invR * (boxmin - r.o);
	float3 ttop = invR * (boxmax - r.o);

	// re-order intersections to find smallest and largest on each axis
	float3 tmin = fminf(ttop, tbot);
	float3 tmax = fmaxf(ttop, tbot);

	// find the largest tmin and the smallest tmax
	float largest_tmin = fmaxf(fmaxf(tmin.x, tmin.y), fmaxf(tmin.x, tmin.z));
	float smallest_tmax = fminf(fminf(tmax.x, tmax.y), fminf(tmax.x, tmax.z));

	*tnear = largest_tmin;
	*tfar = smallest_tmax;

	return smallest_tmax > largest_tmin;
}

// transform vector by matrix (no translation)
__device__
float3 mul(const float3x4 &M, const float3 &v)
{
	float3 r;
	r.x = dot(v, make_float3(M.m[0]));
	r.y = dot(v, make_float3(M.m[1]));
	r.z = dot(v, make_float3(M.m[2]));
	return r;
}

// transform vector by matrix with translation
__device__
float4 mul(const float3x4 &M, const float4 &v)
{
	float4 r;
	r.x = dot(v, M.m[0]);
	r.y = dot(v, M.m[1]);
	r.z = dot(v, M.m[2]);
	r.w = 1.0f;
	return r;
}

__device__ uint volrend_rgbaFloatToInt(float4 rgba)
{
	rgba.x = __saturatef(rgba.x);   // clamp to [0.0, 1.0]
	rgba.y = __saturatef(rgba.y);
	rgba.z = __saturatef(rgba.z);
	rgba.w = __saturatef(rgba.w);
	return (uint(rgba.w * 255) << 24) | (uint(rgba.z * 255) << 16) | (uint(rgba.y * 255) << 8) | uint(rgba.x * 255);
}

__global__ void
d_render(uint *d_output, uint imageW, uint imageH,
float density, float brightness,
float transferOffset, float transferScale)
{
	const int maxSteps = 500;
	const float tstep = 0.01f;
	const float opacityThreshold = 0.95f;
	const float3 boxMin = make_float3(-1.0f, -1.0f, -1.0f);
	const float3 boxMax = make_float3(1.0f, 1.0f, 1.0f);

	uint x = blockIdx.x*blockDim.x + threadIdx.x;
	uint y = blockIdx.y*blockDim.y + threadIdx.y;

	if ((x >= imageW) || (y >= imageH)) return;

	float u = (x / (float)imageW)*2.0f - 1.0f;
	float v = (y / (float)imageH)*2.0f - 1.0f;

	// calculate eye ray in world space
	Ray eyeRay;
	eyeRay.o = make_float3(mul(c_invViewMatrix, make_float4(0.0f, 0.0f, 0.0f, 1.0f)));
	eyeRay.d = normalize(make_float3(u, v, -2.0f));
	eyeRay.d = mul(c_invViewMatrix, eyeRay.d);

	// find intersection with box
	float tnear, tfar;
	int hit = intersectBox(eyeRay, boxMin, boxMax, &tnear, &tfar);

	if (!hit) return;

	if (tnear < 0.0f) tnear = 0.0f;     // clamp to near plane

	// march along ray from front to back, accumulating color
	float4 sum = make_float4(0.0f);
	float t = tnear;
	float3 pos = eyeRay.o + eyeRay.d*tnear;
	float3 step = eyeRay.d*tstep;

	float last_tsdf = tex3D(volumeTex, pos.x*0.5f + 0.5f, pos.y*0.5f + 0.5f, pos.z*0.5f + 0.5f).x;

	for (int i = 0; i<maxSteps; i++)
	{
		// read from 3D texture
		// remap position to [0, 1] coordinates
		float2 sample = tex3D(volumeTex, pos.x*0.5f + 0.5f, pos.y*0.5f + 0.5f, pos.z*0.5f + 0.5f);

		//sample *= 64.0f;    // scale for 10-bit data

#if 0
		// lookup in transfer function texture
		float4 col = tex1D(volumeTransferTex, (sample - transferOffset)*transferScale);
		col.w *= density;

		// "under" operator for back-to-front blending
		//sum = lerp(sum, col, col.w);

		// pre-multiply alpha
		col.x *= col.w;
		col.y *= col.w;
		col.z *= col.w;
		// "over" operator for front-to-back blending
		sum = sum + col*(1.0f - sum.w);

		// exit early if opaque
		if (sum.w > opacityThreshold)
			break;
#else
		//if (sample.x > 1)
		//{
		//	sum = make_float4(1);
		//	break;
		//}

		float tsdf = sample.x;
		if (std::signbit(tsdf) != std::signbit(last_tsdf))
		{
			sum = make_float4(1);
			break;
		}
		else
		{
			last_tsdf = tsdf;
		}

#endif
		t += tstep;

		if (t > tfar) break;

		pos += step;
	}

	sum *= brightness;

	// write output color
	d_output[y*imageW + x] = volrend_rgbaFloatToInt(sum);
}

extern "C"
void setTextureFilterMode(bool bLinearFilter)
{
	volumeTex.filterMode = bLinearFilter ? cudaFilterModeLinear : cudaFilterModePoint;
}

extern "C"
void volume_render_init(void *h_volume, cudaExtent volumeSize)
{
	// create 3D array
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<VolumeType>();
	checkCudaErrors(cudaMalloc3DArray(&d_volumeArray, &channelDesc, volumeSize));

	// copy data to 3D array
	cudaMemcpy3DParms copyParams = { 0 };
	copyParams.srcPtr = make_cudaPitchedPtr(h_volume, volumeSize.width*sizeof(VolumeType), volumeSize.width, volumeSize.height);
	copyParams.dstArray = d_volumeArray;
	copyParams.extent = volumeSize;
	copyParams.kind = cudaMemcpyHostToDevice;
	checkCudaErrors(cudaMemcpy3D(&copyParams));

	// set texture parameters
	volumeTex.normalized = true;                      // access with normalized texture coordinates
	volumeTex.filterMode = cudaFilterModeLinear;      // linear interpolation
	volumeTex.addressMode[0] = cudaAddressModeClamp;  // clamp texture coordinates
	volumeTex.addressMode[1] = cudaAddressModeClamp;

	// bind array to 3D texture
	checkCudaErrors(cudaBindTextureToArray(volumeTex, d_volumeArray, channelDesc));

	// create transfer function texture
	float4 transferFunc[] =
	{
#if 0
		{ 0.0, 0.0, 0.0, 0.0, },
		{ 1.0, 0.0, 0.0, 1.0, },
		{ 1.0, 0.5, 0.0, 1.0, },
		{ 1.0, 1.0, 0.0, 1.0, },
		{ 0.0, 1.0, 0.0, 1.0, },
		{ 0.0, 1.0, 1.0, 1.0, },
		{ 0.0, 0.0, 1.0, 1.0, },
		{ 1.0, 0.0, 1.0, 1.0, },
		{ 0.0, 0.0, 0.0, 0.0, },
#else
#if 0
		{ 0.0, 0.0, 0.0, 0.0, },
		{ 0.0, 0.0, 1.0, 0.1, },
		{ 0.0, 0.3, 1.0, 0.2, },
		{ 0.0, 0.7, 1.0, 0.3, },
		{ 0.1, 1.0, 0.8, 0.4, },
		{ 0.5, 1.0, 0.5, 0.5, },
		{ 0.8, 1.0, 0.1, 0.6, },
		{ 1.0, 0.8, 0.0, 0.7, },
		{ 1.0, 0.4, 0.0, 0.8, },
		{ 0.8, 0.0, 0.0, 0.9, },
		{ 0.5, 0.0, 0.0, 1.0, },
#else
		{ 0.0, 0.0, 0.0, 0.0, },
		{ 1.0, 1.0, 1.0, 1.0, },
#endif
#endif
	};

	cudaChannelFormatDesc channelDesc2 = cudaCreateChannelDesc<float4>();
	cudaArray *d_transferFuncArray;
	checkCudaErrors(cudaMallocArray(&d_transferFuncArray, &channelDesc2, sizeof(transferFunc) / sizeof(float4), 1));
	checkCudaErrors(cudaMemcpyToArray(d_transferFuncArray, 0, 0, transferFunc, sizeof(transferFunc), cudaMemcpyHostToDevice));

	volumeTransferTex.filterMode = cudaFilterModeLinear;
	volumeTransferTex.normalized = true;    // access with normalized texture coordinates
	volumeTransferTex.addressMode[0] = cudaAddressModeClamp;   // wrap texture coordinates

	// Bind the array to the texture
	checkCudaErrors(cudaBindTextureToArray(volumeTransferTex, d_transferFuncArray, channelDesc2));
}

extern "C"
void volume_render_cleanup()
{
	checkCudaErrors(cudaFreeArray(d_volumeArray));
	checkCudaErrors(cudaFreeArray(d_transferFuncArray));
}


extern "C"
void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output, uint imageW, uint imageH,
float density, float brightness, float transferOffset, float transferScale)
{
	d_render << <gridSize, blockSize >> >(d_output, imageW, imageH, density,
		brightness, transferOffset, transferScale);
}

extern "C"
void copyInvViewMatrix(float *invViewMatrix, size_t sizeofMatrix)
{
	checkCudaErrors(cudaMemcpyToSymbol(c_invViewMatrix, invViewMatrix, sizeofMatrix));
}


#endif // #ifndef _VOLUMERENDER_KERNEL_CU_
