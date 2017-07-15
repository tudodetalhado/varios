// DepthBasic-OpenCV.cpp : �������̨Ӧ�ó������ڵ㡣
/****************************************************
������:Kinect V2���������OpenCV��ʾ
��������:win32����̨Ӧ�ó��� x86���� (��������)
		VisualStudio 2012 (��������)
		OpenCV2.4.10 (��ʾ����� vc11��, http://guoming.me/opencv)
		KinectSDK-v2.0-PublicPreview1409-Setup (Kinect SDK�����汾, http://guoming.me/kinect2)
		Windows 8.1 (����ϵͳ)
��������:http://guoming.me/kinect2-depth-opencv
�����ַ:https://github.com/guoming0000/KinectV2/tree/master/DepthBasic-OpenCV

������Ա:С��
����ʱ��:2014-10-20~ 2014-10-21
��ϵ��ʽ:	i@guoming.me (���䣬�Ƽ���ϵ��ʽ)
		http://guoming.me (��վ�������Ѷ��֪ʶ����)
		http://weibo.com/guoming0000 (����΢�������ڷ�����еȿƼ���Ѷ������˽��)
******************************************************/
#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <Kinect.h>// Kinect Header files
using namespace cv;
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//����Kinect������
class Kinect
{
public:
	static const int        cDepthWidth  = 512;
	static const int        cDepthHeight = 424;
	Kinect();
	~Kinect();
	HRESULT					InitKinect();//��ʼ��Kinect
	void					Update();//��������
	void					ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);//����õ�������
private:
	
	IKinectSensor*          m_pKinectSensor;// Current Kinect
	IDepthFrameReader*      m_pDepthFrameReader;// Color reader
	RGBQUAD*                m_pDepthRGBX;

};

//������
int main()
{
	Kinect kinect;
	kinect.InitKinect();
	while(1)
	{
		kinect.Update();
		if(waitKey(1) >= 0)//����������˳�
		{
			break;
		}
	}

	return 0;
}

Kinect::Kinect()
{
	m_pKinectSensor = NULL;
	m_pDepthFrameReader = NULL;
	
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];// create heap storage for color pixel data in RGBX format
}

Kinect::~Kinect()
{
	if (m_pDepthRGBX)
	{
		delete [] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}

	SafeRelease(m_pDepthFrameReader);// done with color frame reader
	
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor
	}
	SafeRelease(m_pKinectSensor);
}

HRESULT	Kinect::InitKinect()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		return E_FAIL;
	}

	return hr;
}

void Kinect::Update()
{
	if (!m_pDepthFrameReader)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
		}

		if (SUCCEEDED(hr))
		{
			ProcessDepth( pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
}

void Kinect::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

			pRGBX->rgbRed   = intensity;
			pRGBX->rgbGreen = intensity;
			pRGBX->rgbBlue  = intensity;

			++pRGBX;
			++pBuffer;
		}
			
		// Draw the data with OpenCV
		Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
		Mat show = DepthImage.clone();
		imshow("DepthImage", show);
	}
}