#ifndef _CUDA_KERNELS_H_
#define _CUDA_KERNELS_H_


#if _MSC_VER // this is defined when compiling with Visual Studio
#define CUDA_KERNELS_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define CUDA_KERNELS_API // XCode does not need annotating exported functions, so define is empty
#endif

#ifdef _WIN32
#include <windows.h>
#endif


#include <cuda_runtime.h>


#ifndef ushort
typedef unsigned short ushort;
#endif
#ifndef uchar
typedef unsigned char uchar;
#endif
#ifndef uint
typedef unsigned int uint;
#endif
#ifndef ulong
typedef unsigned long ulong;
#endif

static int iDivUp(int a, int b)
{
	return (a % b != 0) ? (a / b + 1) : (a / b);
}

extern "C"
{
	void cuda_kernel(float *verts, int vertex_count, float timeElapsed);
	
	void passthrough_texture_uint(
		uint* dOutputImage,
		uint* dInputImage,
		int width, 
		int height, 
		size_t pitch,
		bool invert_channel);

	void passthrough_texture_uchar(
		uchar* dOutputImage,
		uchar* dInputImage,
		int width,
		int height,
		size_t pitch, 
		bool invert_channel);

	void convert_rgba_to_gray(
		uchar4* dInputImage,
		int width,
		int height,
		size_t input_pitch,
		uchar* dOutputImage);


	void raycast_box(
		uint* image_data_ref,
		ushort width,
		ushort height,
		float* camera_to_world_mat3x4,
		ushort3 box_size);
};





#endif // _CUDA_KERNELS_H_