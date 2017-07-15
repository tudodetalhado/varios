
#include <QApplication>
#include <QKeyEvent>
#include <iostream>
#include <fstream>
#include <cuda_runtime.h>
#include "helper_cuda.h"
#include "helper_image.h"
#include "CudaKernels/CudaKernels.h"
#include "QImageWidget.h"


template<typename PixelType>
void run_test(const QImage& inputImage, QImage& outputPassImage, QImage& outputInvImage)
{
	PixelType* hImage = (PixelType*)(inputImage.bits());
	unsigned int width = inputImage.width();
	unsigned int height = inputImage.height();

	size_t pitch;

	PixelType* dInputImage = nullptr;
	// copy image data to array
	checkCudaErrors(cudaMallocPitch(&dInputImage, &pitch, sizeof(PixelType) * width, height));
	checkCudaErrors(cudaMemcpy2D(
		dInputImage,
		pitch,
		hImage,
		sizeof(PixelType) * width,
		sizeof(PixelType) * width,
		height,
		cudaMemcpyHostToDevice));



	PixelType* dOutputImage;
	checkCudaErrors(cudaMallocPitch(
		&dOutputImage,
		&pitch,
		width * sizeof(PixelType),
		height));


	if (sizeof(PixelType) == 1)
		passthrough_texture_uchar((uchar*)dOutputImage, (uchar*)dInputImage, width, height, pitch, false);
	else
		passthrough_texture_uint((uint*)dOutputImage, (uint*)dInputImage, width, height, pitch, false);


	cudaMemcpy2D(
		outputPassImage.bits(),
		sizeof(PixelType) * width,
		dOutputImage,
		pitch,
		sizeof(PixelType) * width,
		height,
		cudaMemcpyDeviceToHost);


	if (sizeof(PixelType) == 1)
		passthrough_texture_uchar((uchar*)dOutputImage, (uchar*)dInputImage, width, height, pitch, true);
	else
		passthrough_texture_uint((uint*)dOutputImage, (uint*)dInputImage, width, height, pitch, true);


	cudaMemcpy2D(
		outputInvImage.bits(),
		sizeof(PixelType) * width,
		dOutputImage,
		pitch,
		sizeof(PixelType) * width,
		height,
		cudaMemcpyDeviceToHost);


	checkCudaErrors(cudaFree(dInputImage));
	checkCudaErrors(cudaFree(dOutputImage));
}



template<typename InputPixelType, typename OutputPixelType>
void run_test_rgba2gray(const QImage& inputImage, QImage& outputImage)
{
	InputPixelType* hImage = (InputPixelType*)(inputImage.bits());
	unsigned int width = inputImage.width();
	unsigned int height = inputImage.height();

	size_t input_pitch, output_pitch;

	InputPixelType* dInputImage = nullptr;
	// copy image data to array
	checkCudaErrors(cudaMallocPitch(&dInputImage, &input_pitch, sizeof(InputPixelType) * width, height));
	checkCudaErrors(cudaMemcpy2D(
		dInputImage,
		input_pitch,
		hImage,
		sizeof(InputPixelType) * width,
		sizeof(InputPixelType) * width,
		height,
		cudaMemcpyHostToDevice));



	OutputPixelType* dOutputImage;
	checkCudaErrors(cudaMallocPitch(
		&dOutputImage,
		&output_pitch,
		width * sizeof(OutputPixelType),
		height));


	//passthrough_texture_uchar((uchar*)dOutputImage, (uchar*)dInputImage, width, height, pitch, false);
	//passthrough_texture_uint((uint*)dOutputImage, (uint*)dInputImage, width, height, pitch, false);
	convert_rgba_to_gray((uchar4*)dInputImage, width, height, input_pitch, (uchar*)dOutputImage);


	cudaMemcpy2D(
		outputImage.bits(),
		sizeof(OutputPixelType) * width,
		dOutputImage,
		output_pitch,
		sizeof(OutputPixelType) * width,
		height,
		cudaMemcpyDeviceToHost);


	checkCudaErrors(cudaFree(dInputImage));
	checkCudaErrors(cudaFree(dOutputImage));
}



int test_passthrough(int argc, char **argv)
{
	QApplication app(argc, argv);
	app.setApplicationName("Qt Cuda Texture Example");

	QImage inputImage;
	if (!inputImage.load(argv[1]))
	{
		std::cout << "Error: Could not load file image: " << argv[1] << std::endl;
		return EXIT_FAILURE;
	}

	QImage outputInvertImage = inputImage;
	QImage outputPassthroughImage = inputImage;


	if (inputImage.format() == QImage::Format_Indexed8)
		run_test<uchar>(inputImage, outputPassthroughImage, outputInvertImage);
	else
		run_test<uint>(inputImage, outputPassthroughImage, outputInvertImage);

	QImageWidget inputWidget;
	inputWidget.setImage(inputImage);
	inputWidget.move(0, 0);
	inputWidget.setWindowTitle("Input");
	inputWidget.show();

	QImageWidget outputPassWidget;
	outputPassWidget.setImage(outputPassthroughImage);
	outputPassWidget.move(0, inputWidget.height());
	outputPassWidget.setWindowTitle("Output Passthrough");
	outputPassWidget.show();

	QImageWidget outputInvWidget;
	outputInvWidget.setImage(outputInvertImage);
	outputInvWidget.move(inputWidget.width(), inputWidget.height());
	outputInvWidget.setWindowTitle("Output Inverted");
	outputInvWidget.show();

	return app.exec();
}



int test_conversion(int argc, char **argv)
{
	QApplication app(argc, argv);
	app.setApplicationName("Qt Cuda Texture Example");

	QImage inputImage;
	if (!inputImage.load(argv[1]))
	{
		std::cout << "Error: Could not load file image: " << argv[1] << std::endl;
		return EXIT_FAILURE;
	}

	if (inputImage.format() == QImage::Format_Indexed8)
	{
		std::cerr << "Warning: Input image is already gray scale. Conversion is aborted." << std::endl;
		return 0;
	}

	// Creating output image
	QVector<QRgb> colorTable;
	for (int i = 0; i < 256; ++i)
		colorTable.push_back(qRgb(i, i, i));
	QImage outputImage(inputImage.width(), inputImage.height(), QImage::Format::Format_Indexed8);
	outputImage.setColorTable(colorTable);

	

	if (inputImage.format() != QImage::Format::Format_ARGB32)
		inputImage.convertToFormat(QImage::Format::Format_ARGB32);

	run_test_rgba2gray<uchar4, uchar>(inputImage, outputImage);

	QImageWidget inputWidget;
	inputWidget.setImage(inputImage);
	inputWidget.move(0, 0);
	inputWidget.setWindowTitle("Input");
	inputWidget.show();

	QImageWidget outputWidget;
	outputWidget.setImage(outputImage);
	outputWidget.move(inputWidget.width(), 0);
	outputWidget.setWindowTitle("Output");
	outputWidget.show();

	return app.exec();
}




int main(int argc, char **argv)
{
	if (argc < 1)
	{
		std::cerr << "Usage: QtCudaTexture.exe ../../data/lena.jpg" << std::endl;
		return EXIT_FAILURE;
	}
	
	
	test_conversion(argc, argv);
	test_passthrough(argc, argv);	
}
