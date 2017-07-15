// Created by Heresy @ 2015/02/10
// Blog Page: https://kheresy.wordpress.com/2015/02/11/k4w-v2-part-6-draw-with-opengl/
// This sample is used to show Kinect data in triangles form.

// Standard Library
#include <array>
#include <iostream>

// freeglut header
#include <GL/freeglut.h>

// Kinect for Windows SDK Header
#include <Kinect.h>

// for OpenGL camera navigation
#include "../common/OpenGLCamera.h"

using namespace std;

// global objects
IKinectSensor*		pSensor				= nullptr;
IColorFrameReader*	pColorFrameReader	= nullptr;
IDepthFrameReader*	pDepthFrameReader	= nullptr;
ICoordinateMapper*	pCoordinateMapper	= nullptr;

int		iColorWidth		= 0,
		iColorHeight	= 0;
UINT	uDepthPointNum		= 0;
UINT	uColorPointNum		= 0;
UINT	uColorBufferSize	= 0;

UINT16*	pDepthBuffer = nullptr;
BYTE*	pColorBuffer = nullptr;
CameraSpacePoint*	pCSPoints = nullptr;

unsigned int*	pIndex = nullptr;
unsigned int	uTriangles = 0;

SimpleCamera g_Camera;

// function to build triangle
inline bool BuildTriangle(unsigned int& iIdxCounter, unsigned int v1, unsigned int v2, unsigned int v3, float th = 0.05f)
{
	const CameraSpacePoint	&rP1 = pCSPoints[v1],
							&rP2 = pCSPoints[v2],
							&rP3 = pCSPoints[v3];

	if (rP1.Z > 0 && rP2.Z > 0 && rP3.Z > 0)
	{
		if (abs(rP1.Z - rP2.Z) > th ||
			abs(rP1.Z - rP3.Z) > th ||
			abs(rP2.Z - rP3.Z) > th)
			return false;

		pIndex[iIdxCounter++] = v1;
		pIndex[iIdxCounter++] = v2;
		pIndex[iIdxCounter++] = v3;

		return true;
	}
	return false;
}

// glut display function(draw)
void display()
{
	// clear previous screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// update vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, pCSPoints);

	// update color array
	glEnableClientState(GL_COLOR_ARRAY);
	glColorPointer(4, GL_UNSIGNED_BYTE, 0, pColorBuffer);

	// draw
	glDrawElements(GL_TRIANGLES, uTriangles, GL_UNSIGNED_INT, pIndex);

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	// Coordinate
	glLineWidth(5.0f);
	glBegin(GL_LINES);
		glColor3ub(255, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);

		glColor3ub(0, 255, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);

		glColor3ub(0, 0, 255);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
	glEnd();

	// swap buffer
	glutSwapBuffers();
}

// glut idle function
void idle()
{
	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
	{
		pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorBuffer, ColorImageFormat_Rgba);

		pCFrame->Release();
		pCFrame = nullptr;
	}

	// Read depth data
	IDepthFrame* pDFrame = nullptr;
	if (pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
	{
		pDFrame->CopyFrameDataToArray(uDepthPointNum, pDepthBuffer);

		pDFrame->Release();
		pDFrame = nullptr;

		// map to camera space
		pCoordinateMapper->MapColorFrameToCameraSpace(uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);

		// build index array of triangles
		uTriangles = 0;
		for (int y = 0; y < iColorHeight - 1; ++y)
		{
			for (int x = 0; x < iColorWidth - 1; ++x)
			{
				unsigned int idx = x + y * iColorWidth;
				BuildTriangle(uTriangles, idx, idx + 1, idx + iColorWidth);
				BuildTriangle(uTriangles, idx + 1, idx + iColorWidth + 1, idx + iColorWidth);
			}
		}

		// redraw
		glutPostRedisplay();
	}
}

// glut keyboard function
void keyboard(unsigned char key, int x, int y)
{
	float fSpeed = 0.1f;
	switch (key)
	{
	case VK_ESCAPE:
		glutExit();

	case 's':
		g_Camera.MoveForward(-fSpeed);
		break;

	case 'w':
		g_Camera.MoveForward(fSpeed);
		break;

	case 'a':
		g_Camera.MoveSide(-fSpeed);
		break;

	case 'd':
		g_Camera.MoveSide(fSpeed);
		break;

	case 'z':
		g_Camera.MoveUp(-fSpeed);
		break;

	case 'x':
		g_Camera.MoveUp(fSpeed);
		break;
	}

}

// glut special keyboard function
void specialKey(int key, int x, int y)
{
	float fRotateScale = 0.01f;
	switch (key)
	{
	case GLUT_KEY_DOWN:
		g_Camera.RotateUp(-fRotateScale);
		break;

	case GLUT_KEY_UP:
		g_Camera.RotateUp(fRotateScale);
		break;

	case GLUT_KEY_RIGHT:
		g_Camera.RotateSide(fRotateScale);
		break;

	case GLUT_KEY_LEFT:
		g_Camera.RotateSide(-fRotateScale);
		break;
	}
}

void reshape(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0, (float)w / h, 0.01, 50.0);

	g_Camera.SetCamera();

	glViewport(0, 0, w, h);
}

void ExitFunction()
{
	// release buffer
	delete[] pDepthBuffer;
	delete[] pColorBuffer;
	delete[] pCSPoints;
	delete[] pIndex;

	// release coordinate mapper
	pCoordinateMapper->Release();
	pCoordinateMapper = nullptr;

	// release frame reader
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	pSensor->Release();
	pSensor = nullptr;
}

int main(int argc, char** argv)
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(BYTE);

			pCSPoints = new CameraSpacePoint[uColorPointNum];
			pIndex = new unsigned int[6 * uColorPointNum];
			pColorBuffer = new BYTE[4 * uColorPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			int	iDepthWidth = 0,
				iDepthHeight = 0;
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
			pDepthBuffer = new UINT16[uDepthPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	// 5. initial glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(640, 480);
	glutCreateWindow("Kinect OpenGL 3D Mesh");

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	// default camera
	g_Camera.vCenter = Vector3(0.0, 0.0, 1.0);
	g_Camera.vPosition = Vector3(0.0, 0.0, -2.0);
	g_Camera.vUpper = Vector3(0.0, 1.0, 0.0);

	// register glut callback functions
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKey);

	// STD Exit function
	atexit(ExitFunction);

	// start
	glutMainLoop();

	return 0;
}
