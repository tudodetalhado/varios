#include "CudaKernels.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cuda_runtime.h>
#include <cublas_v2.h>
#include <curand.h>
#include <helper_cuda.h>
#include <helper_math.h>
#include <thrust/device_vector.h>


__device__ uint rgba_float_to_int(float4 rgba)
{
	rgba.x = __saturatef(fabs(rgba.x));   // clamp to [0.0, 1.0]
	rgba.y = __saturatef(fabs(rgba.y));
	rgba.z = __saturatef(fabs(rgba.z));
	rgba.w = __saturatef(fabs(rgba.w));
	return (uint(rgba.w * 255.0f) << 24) | (uint(rgba.z * 255.0f) << 16) | (uint(rgba.y * 255.0f) << 8) | uint(rgba.x * 255.0f);
}

struct Ray
{
	float3 origin;   // origin
	float3 direction;   // direction
};

// intersect ray with a box
// http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm
__device__ int intersectBox(Ray r, float3 boxmin, float3 boxmax, float *tnear, float *tfar)
{
	// compute intersection of ray with all six bbox planes
	float3 invR = make_float3(1.0f) / r.direction;
	float3 tbot = invR * (boxmin - r.origin);
	float3 ttop = invR * (boxmax - r.origin);

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


typedef struct
{
	float4 m[3];
} float3x4;
__constant__ float3x4 camera_to_world_dev_matrix;  // inverse view matrix


// transform vector by matrix (no translation)
__device__ float3 mul(const float3x4 &M, const float3 &v)
{
	float3 r;
	r.x = dot(v, make_float3(M.m[0]));
	r.y = dot(v, make_float3(M.m[1]));
	r.z = dot(v, make_float3(M.m[2]));
	return r;
}

// transform vector by matrix with translation
__device__ float4 mul(const float3x4 &M, const float4 &v)
{
	float4 r;
	r.x = dot(v, M.m[0]);
	r.y = dot(v, M.m[1]);
	r.z = dot(v, M.m[2]);
	r.w = 1.0f;
	return r;
}

extern "C"
{

	__global__ void update_vb(float *d_verts_ptr, int vertex_count, float timeElapsed)
	{
		const unsigned long long int threadId = blockIdx.x * blockDim.x + threadIdx.x;

		if (threadId < vertex_count * 4)
		{
			float valx = d_verts_ptr[threadId * 4 + 0];
			float valy = d_verts_ptr[threadId * 4 + 1];
			float valz = d_verts_ptr[threadId * 4 + 2];


			d_verts_ptr[threadId * 4 + 0] = valx * timeElapsed;
			d_verts_ptr[threadId * 4 + 1] = valy * timeElapsed;
			d_verts_ptr[threadId * 4 + 2] = valz * timeElapsed;
		}
	}

	void cuda_kernel(float *d_verts_ptr, int vertex_count, float timeElapsed)
	{
		if (vertex_count > 1024)
			update_vb << <vertex_count / 1024 + 1, 1024 >> >(d_verts_ptr, vertex_count, timeElapsed);
		else
			update_vb << <1, vertex_count >> >(d_verts_ptr, vertex_count, timeElapsed);
	}






	__global__ void	raycast_box_kernel(
		uint* out_image,
		ushort image_width,
		ushort image_height,
		ushort3 box_size)
	{
		ulong x = blockIdx.x * blockDim.x + threadIdx.x;
		ulong y = blockIdx.y * blockDim.y + threadIdx.y;

		if (x >= image_width || y >= image_height)
			return;

		// Convert from image space (in pixels) to screen space
		float u = (x / (float)image_width) * 2.0f - 1.0f;
		float v = (y / (float)image_height) * 2.0f - 1.0f;
		Ray eye_ray;
		eye_ray.origin = make_float3(mul(camera_to_world_dev_matrix, make_float4(0.0f, 0.0f, 0.0f, 1.0f)));
		float3 screen_coord = normalize(make_float3(u, -v, -2.0f));
		eye_ray.direction = mul(camera_to_world_dev_matrix, screen_coord);

		// find intersection with box
		const float3 boxMin = make_float3(-1.0f, -1.0f, -1.0f);
		const float3 boxMax = make_float3(1.0f, 1.0f, 1.0f);
		float tnear, tfar;
		
		if (intersectBox(eye_ray, boxMin, boxMax, &tnear, &tfar))
			out_image[y * image_width + x] = rgba_float_to_int(make_float4(0.f, 0.5f, 0.5f, 1.f));
		else
			out_image[y * image_width + x] = rgba_float_to_int(make_float4(0.f, 0.0f, 0.0f, 1.f));
	}

	void raycast_box(
		uint* image_data_ref,
		ushort width,
		ushort height,
		float* camera_to_world_mat3x4,
		ushort3 box_size)
	{
		checkCudaErrors(cudaMemcpyToSymbol(camera_to_world_dev_matrix, camera_to_world_mat3x4, sizeof(float4) * 3));

		const dim3 threads_per_block(16, 16);
		dim3 num_blocks = dim3(iDivUp(width, threads_per_block.x), iDivUp(height, threads_per_block.y));;

		raycast_box_kernel << < num_blocks, threads_per_block >> >(
			image_data_ref,
			width,
			height,
			box_size
			);

		checkCudaErrors(cudaDeviceSynchronize());
	}

};