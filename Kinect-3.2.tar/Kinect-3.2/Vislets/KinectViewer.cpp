/***********************************************************************
KinectViewer - Vislet to draw 3D reconstructions captured from a Kinect
device in 3D space.
Copyright (c) 2010-2016 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#include "Vislets/KinectViewer.h"

#include <string.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/FunctionCalls.h>
#include <Misc/PrintInteger.h>
#include <Misc/ValueCoder.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <Cluster/OpenPipe.h>
#include <Math/Constants.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/GeometryMarshallers.h>
#include <GL/gl.h>
#include <GL/GLTransformationWrappers.h>
#include <Video/Config.h>
#include <Vrui/Vrui.h>
#include <Vrui/InputDevice.h>
#include <Vrui/DisplayState.h>
#include <Vrui/VisletManager.h>
#include <Vrui/OpenFile.h>
#include <Kinect/Config.h>
#include <Kinect/FunctionCalls.h>
#include <Kinect/Camera.h>
#include <Kinect/CameraV2.h>
#if KINECT_CONFIG_HAVE_LIBREALSENSE
#include <Kinect/CameraRealSense.h>
#endif
#include <Kinect/DepthFrameReader.h>
#if VIDEO_CONFIG_HAVE_THEORA
#include <Kinect/LossyDepthFrameReader.h>
#endif
#include <Kinect/ColorFrameReader.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/FrameSaver.h>

/************************************
Methods of class KinectViewerFactory:
************************************/

KinectViewerFactory::KinectViewerFactory(Vrui::VisletManager& visletManager)
	:Vrui::VisletFactory("KinectViewer",visletManager)
	{
	#if 0
	/* Insert class into class hierarchy: */
	Vrui::VisletFactory* visletFactory=visletManager.loadClass("Vislet");
	visletFactory->addChildClass(this);
	addParentClass(visletFactory);
	#endif
	
	/* Initialize the KinectViewer tool classes: */
	KinectViewer::PauseViewerTool::initClass();
	KinectViewer::MapTextureTool::initClass();
	
	/* Set vislet class' factory pointer: */
	KinectViewer::factory=this;
	}

KinectViewerFactory::~KinectViewerFactory(void)
	{
	/* Reset vislet class' factory pointer: */
	KinectViewer::factory=0;
	}

Vrui::Vislet* KinectViewerFactory::createVislet(int numArguments,const char* const arguments[]) const
	{
	return new KinectViewer(numArguments,arguments);
	}

void KinectViewerFactory::destroyVislet(Vrui::Vislet* vislet) const
	{
	delete vislet;
	}

extern "C" void resolveKinectViewerDependencies(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	#if 0
	/* Load base classes: */
	manager.loadClass("Vislet");
	#endif
	}

extern "C" Vrui::VisletFactory* createKinectViewerFactory(Plugins::FactoryManager<Vrui::VisletFactory>& manager)
	{
	/* Get pointer to vislet manager: */
	Vrui::VisletManager* visletManager=static_cast<Vrui::VisletManager*>(&manager);
	
	/* Create factory object and insert it into class hierarchy: */
	KinectViewerFactory* kinectViewerFactory=new KinectViewerFactory(*visletManager);
	
	/* Return factory object: */
	return kinectViewerFactory;
	}

extern "C" void destroyKinectViewerFactory(Vrui::VisletFactory* factory)
	{
	delete factory;
	}

/***************************************
Methods of class KinectViewer::Renderer:
***************************************/

KinectViewer::Renderer::~Renderer(void)
	{
	/* Destroy the projector: */
	delete projector;
	}

void KinectViewer::Renderer::applyPreTransform(const Kinect::FrameSource::ExtrinsicParameters& preTransform)
	{
	Kinect::FrameSource::ExtrinsicParameters ext=projector->getProjectorTransform();
	ext.leftMultiply(preTransform);
	projector->setExtrinsicParameters(ext);
	}

void KinectViewer::Renderer::glRenderAction(GLContextData& contextData) const
	{
	/* Draw the current 3D video frame: */
	projector->glRenderAction(contextData);
	}

/*******************************************
Methods of class KinectViewer::LiveRenderer:
*******************************************/

void KinectViewer::LiveRenderer::colorStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	if(paused)
		return;
	
	/* Forward color frame to the projector: */
	projector->setColorFrame(frameBuffer);
	
	Vrui::requestUpdate();
	
	if(frameSaver!=0)
		{
		/* Time-stamp the incoming frame: */
		Kinect::FrameBuffer saveFrame(frameBuffer);
		saveFrame.timeStamp=timeStamp;
		
		/* Forward the time-stamped frame to the frame saver: */
		frameSaver->saveColorFrame(saveFrame);
		}
	}

void KinectViewer::LiveRenderer::depthStreamingCallback(const Kinect::FrameBuffer& frameBuffer)
	{
	// DEBUGGING
	std::cout<<'D'<<std::flush;
	
	if(paused)
		return;
	
	/* Forward depth frame to the projector: */
	projector->setDepthFrame(frameBuffer);
	
	#if KINECT_CONFIG_USE_SHADERPROJECTOR
	Vrui::requestUpdate();
	#endif
	
	if(frameSaver!=0)
		{
		/* Time-stamp the incoming frame: */
		Kinect::FrameBuffer saveFrame(frameBuffer);
		saveFrame.timeStamp=timeStamp;
		
		/* Forward the time-stamped frame to the frame saver: */
		frameSaver->saveDepthFrame(saveFrame);
		}
	}

#if !KINECT_CONFIG_USE_SHADERPROJECTOR

void KinectViewer::LiveRenderer::meshStreamingCallback(const Kinect::MeshBuffer& meshBuffer)
	{
	// DEBUGGING
	std::cout<<'M'<<std::flush;
	
	if(paused)
		return;
	
	Vrui::requestUpdate();
	}

#endif

KinectViewer::LiveRenderer::LiveRenderer(Kinect::FrameSource* sSource)
	:source(sSource),started(false),paused(false),
	 frameSaver(0),timeStamp(0.0)
	{
	/* Create the projector: */
	projector=new Kinect::ProjectorType(*source);
	}

KinectViewer::LiveRenderer::~LiveRenderer(void)
	{
	if(started)
		{
		/* Stop streaming: */
		source->stopStreaming();
		#if !KINECT_CONFIG_USE_SHADERPROJECTOR
		projector->stopStreaming();
		#endif
		}
	
	/* Destroy the frame source: */
	delete source;
	
	/* Destroy the frame saver: */
	delete frameSaver;
	}

void KinectViewer::LiveRenderer::startStreaming(const Kinect::FrameSource::Time& timeBase)
	{
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	
	/* Hook this renderer into the projector's mesh callback: */
	projector->startStreaming(Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::meshStreamingCallback));
	
	#endif
	
	/* Hook this renderer into the frame source and start streaming: */
	source->setTimeBase(timeBase);
	source->startStreaming(Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::colorStreamingCallback),Misc::createFunctionCall(this,&KinectViewer::LiveRenderer::depthStreamingCallback));
	
	// DEBUGGING
	std::cout<<"Started streaming"<<std::endl;
	
	started=true;
	}

void KinectViewer::LiveRenderer::frame(double newTimeStamp)
	{
	// DEBUGGING
	std::cout<<'F'<<std::flush;
	
	/* Update the projector: */
	projector->updateFrames();
	
	/* Update the time stamp: */
	timeStamp=newTimeStamp;
	}

void KinectViewer::LiveRenderer::saveStreams(const std::string& saveFileName)
	{
	/* Ignore request if already streaming: */
	if(!started)
		{
		std::string colorFileName=saveFileName;
		colorFileName.append(".color");
		std::string depthFileName=saveFileName;
		depthFileName.append(".depth");
		frameSaver=new Kinect::FrameSaver(*source,colorFileName.c_str(),depthFileName.c_str());
		}
	}

/**********************************************
Methods of class KinectViewer::TrackedRenderer:
**********************************************/

KinectViewer::TrackedRenderer::TrackedRenderer(Kinect::FrameSource* sSource,Vrui::InputDevice* sTrackingDevice)
	:LiveRenderer(sSource),
	 latency(0.0),
	 trackingDevice(sTrackingDevice),
	 trackingBufferSize(2048),timeStampBuffer(new double[trackingBufferSize]),trackingBuffer(new Vrui::TrackerState[trackingBufferSize]),
	 numTrackingBufferEntries(0),tail(0),
	 meshTimeStamp(0.0),meshTrackerState(Vrui::TrackerState::identity)
	{
	}

KinectViewer::TrackedRenderer::~TrackedRenderer(void)
	{
	/* Delete the tracking buffer: */
	delete[] timeStampBuffer;
	delete[] trackingBuffer;
	}

void KinectViewer::TrackedRenderer::startStreaming(const Kinect::FrameSource::Time& timeBase)
	{
	/* Call the base class method: */
	LiveRenderer::startStreaming(timeBase);
	
	/* Remember the time base: */
	sourceTimeBase=timeBase;
	}

void KinectViewer::TrackedRenderer::frame(double newTimeStamp)
	{
	/* Put the current device position/orientation into the ring buffer: */
	Kinect::FrameSource::Time now;
	timeStampBuffer[tail]=double(now-sourceTimeBase);
	trackingBuffer[tail]=trackingDevice->getTransformation();
	if(numTrackingBufferEntries<trackingBufferSize)
		++numTrackingBufferEntries;
	if(++tail==trackingBufferSize)
		tail=0;
	
	/* Call the base class method: */
	LiveRenderer::frame(newTimeStamp);
	
	/* Check if the projector has a new mesh to display: */
	double newMeshTimeStamp=projector->getMeshTimeStamp();
	if(meshTimeStamp!=newMeshTimeStamp)
		{
		if(numTrackingBufferEntries>0U)
			{
			/* Find the tracking device state whose time stamp is closest to the new mesh's capture time: */
			double exposureTime=newMeshTimeStamp-latency;
			size_t l=tail+trackingBufferSize-numTrackingBufferEntries;
			size_t r=tail+trackingBufferSize;
			while(r-l>1U)
				{
				size_t m=(l+r)>>1;
				
				if(timeStampBuffer[m%trackingBufferSize]<=exposureTime)
					l=m;
				else
					r=m;
				}
			
			/* Store the tracked device's state: */
			if(r<tail+trackingBufferSize&&timeStampBuffer[r%trackingBufferSize]-exposureTime<=exposureTime-timeStampBuffer[l%trackingBufferSize])
				meshTrackerState=trackingBuffer[r%trackingBufferSize];
			else
				meshTrackerState=trackingBuffer[l%trackingBufferSize];
			}
		
		meshTimeStamp=newMeshTimeStamp;
		}
	}

void KinectViewer::TrackedRenderer::glRenderAction(GLContextData& contextData) const
	{
	/* Transform the projector by the stored transformation: */
	glPushMatrix();
	glMultMatrix(meshTrackerState);
	
	/* Draw the current 3D video frame: */
	projector->glRenderAction(contextData);
	
	glPopMatrix();
	}

/**********************************************
Methods of class KinectViewer::SynchedRenderer:
**********************************************/

void* KinectViewer::SynchedRenderer::colorReaderThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read color frames: */
	while(true)
		{
		/* Read the next color frame: */
		Kinect::FrameBuffer nextFrame=colorReader->readNextFrame();
		
		/* Put the new color frame into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numColorFrames==numQueueSlots)
			colorFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentColorFrame=(mostRecentColorFrame+1)%numQueueSlots;
		colorFrames[mostRecentColorFrame]=nextFrame;
		if(++numColorFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextFrame.timeStamp>=Math::Constants<double>::max)
			break;
		}
	
	return 0;
	}

void* KinectViewer::SynchedRenderer::depthReaderThreadMethod(void)
	{
	Threads::Thread::setCancelState(Threads::Thread::CANCEL_ENABLE);
	// Threads::Thread::setCancelType(Threads::Thread::CANCEL_ASYNCHRONOUS);
	
	/* Read depth frames: */
	while(true)
		{
		/* Read the next depth frame: */
		Kinect::FrameBuffer nextFrame=depthReader->readNextFrame();
		
		#if KINECT_CONFIG_USE_SHADERPROJECTOR
		
		/* Put the new depth frame into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numDepthFrames==numQueueSlots)
			depthFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentDepthFrame=(mostRecentDepthFrame+1)%numQueueSlots;
		depthFrames[mostRecentDepthFrame]=nextFrame;
		if(++numDepthFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextFrame.timeStamp>=Math::Constants<double>::max)
			break;
		
		#else
		
		/* Process the next depth frame into a mesh: */
		Kinect::MeshBuffer nextMesh;
		projector->processDepthFrame(nextFrame,nextMesh);
		
		/* Put the new depth frame and mesh into the queue: */
		{
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numDepthFrames==numQueueSlots)
			depthFrameQueueFullCond.wait(frameQueueMutex);
		mostRecentDepthFrame=(mostRecentDepthFrame+1)%numQueueSlots;
		depthFrames[mostRecentDepthFrame]=nextFrame;
		meshes[mostRecentDepthFrame]=nextMesh;
		if(++numDepthFrames==1)
			frameQueuesEmptyCond.broadcast();
		}
		
		if(nextMesh.timeStamp>=Math::Constants<double>::max)
			break;
		
		#endif
		}
	
	return 0;
	}

KinectViewer::SynchedRenderer::SynchedRenderer(const std::string& fileName)
	:colorReader(0),depthReader(0),
	 started(false),
	 timeStamp(0.0),
	 numColorFrames(0),mostRecentColorFrame(0),
	 numDepthFrames(0),mostRecentDepthFrame(0)
	{
	/* Open the color file: */
	std::string colorFileName=fileName;
	colorFileName.append(".color");
	colorFile=Vrui::openFile(colorFileName.c_str());
	colorFile->setEndianness(Misc::LittleEndian);
	
	/* Open the depth file: */
	std::string depthFileName=fileName;
	depthFileName.append(".depth");
	depthFile=Vrui::openFile(depthFileName.c_str());
	depthFile->setEndianness(Misc::LittleEndian);
	
	/* Read the files' format version numbers: */
	unsigned int colorFormatVersion=colorFile->read<Misc::UInt32>();
	unsigned int depthFormatVersion=depthFile->read<Misc::UInt32>();
	if(colorFormatVersion>1||depthFormatVersion>5)
		Misc::throwStdErr("KinectViewer::SynchedRenderer: Unsupported 3D video file format");
	
	/* Check if there are per-pixel depth correction coefficients: */
	Kinect::FrameSource::DepthCorrection* depthCorrection=0;
	if(depthFormatVersion>=4)
		{
		/* Read new B-spline based depth correction parameters: */
		depthCorrection=new Kinect::FrameSource::DepthCorrection(*depthFile);
		}
	else
		{
		if(depthFormatVersion>=2&&depthFile->read<Misc::UInt8>()!=0)
			{
			/* Skip the depth correction buffer: */
			Misc::SInt32 size[2];
			depthFile->read<Misc::SInt32>(size,2);
			depthFile->skip<Misc::Float32>(size[1]*size[0]*2);
			}
		
		/* Create a dummy depth correction object: */
		int numSegments[2]={1,1};
		depthCorrection=new Kinect::FrameSource::DepthCorrection(0,numSegments);
		}
	
	/* Check if the depth stream uses lossy compression: */
	bool depthIsLossy=depthFormatVersion>=3&&depthFile->read<Misc::UInt8>()!=0;
	
	/* Read the color and depth projections from their respective files: */
	Kinect::FrameSource::IntrinsicParameters ips;
	
	/* Check if the depth camera has lens distortion correction parameters: */
	if(depthFormatVersion>=5)
		{
		/* Read the depth camera's lens distortion correction parameters: */
		ips.depthLensDistortion.read(*depthFile);
		}
	
	/* Read the intrinsic projection matrices: */
	ips.colorProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*colorFile);
	ips.depthProjection=Misc::Marshaller<Kinect::FrameSource::IntrinsicParameters::PTransform>::read(*depthFile);
	
	/* Read the camera transformation from the depth file: */
	Kinect::FrameSource::ExtrinsicParameters eps;
	eps=Misc::Marshaller<Kinect::FrameSource::ExtrinsicParameters>::read(*depthFile);
	
	/* Create the color and depth readers: */
	colorReader=new Kinect::ColorFrameReader(*colorFile);
	if(depthIsLossy)
		{
		#if VIDEO_CONFIG_HAVE_THEORA
		depthReader=new Kinect::LossyDepthFrameReader(*depthFile);
		#else
		delete colorReader;
		Misc::throwStdErr("KinectViewer::SynchedRenderer: Lossy depth compression not supported due to lack of Theora library");
		#endif
		}
	else
		depthReader=new Kinect::DepthFrameReader(*depthFile);
	
	/* Create and initialize the projector: */
	projector=new Kinect::ProjectorType();
	projector->setDepthFrameSize(depthReader->getSize());
	projector->setDepthCorrection(depthCorrection);
	projector->setIntrinsicParameters(ips);
	projector->setExtrinsicParameters(eps);
	
	/* Clean up: */
	delete depthCorrection;
	}

KinectViewer::SynchedRenderer::~SynchedRenderer(void)
	{
	if(started)
		{
		/* Shut down the depth and color reader threads: */
		colorReaderThread.cancel();
		depthReaderThread.cancel();
		colorReaderThread.join();
		depthReaderThread.join();
		}
	
	/* Delete the color and depth readers: */
	delete colorReader;
	delete depthReader;
	}

void KinectViewer::SynchedRenderer::startStreaming(const Kinect::FrameSource::Time& timeBase)
	{
	/* Start the color and depth reader threads: */
	colorReaderThread.start(this,&KinectViewer::SynchedRenderer::colorReaderThreadMethod);
	depthReaderThread.start(this,&KinectViewer::SynchedRenderer::depthReaderThreadMethod);
	
	started=true;
	}

void KinectViewer::SynchedRenderer::frame(double newTimeStamp)
	{
	timeStamp=newTimeStamp;
	
	/* Wait until the next frame is newer than the new time step: */
	bool newColor=false;
	Kinect::FrameBuffer currentColorFrame;
	bool newDepth=false;
	Kinect::FrameBuffer currentDepthFrame;
	#if !KINECT_CONFIG_USE_SHADERPROJECTOR
	Kinect::MeshBuffer currentMesh;
	#endif
	while(nextColorFrame.timeStamp<=timeStamp||nextDepthFrame.timeStamp<=timeStamp)
		{
		/* Check if both frame queues are empty: */
		Threads::Mutex::Lock frameQueueLock(frameQueueMutex);
		while(numColorFrames==0&&numDepthFrames==0)
			frameQueuesEmptyCond.wait(frameQueueMutex);
		
		/* Advance in the color frame queue: */
		while(numColorFrames>0&&nextColorFrame.timeStamp<=timeStamp)
			{
			newColor=true;
			currentColorFrame=nextColorFrame;
			nextColorFrame=colorFrames[(mostRecentColorFrame-numColorFrames+numQueueSlots+1)%numQueueSlots];
			if(--numColorFrames==numQueueSlots-1)
				colorFrameQueueFullCond.broadcast();
			}
		
		/* Advance in the depth frame queue: */
		while(numDepthFrames>0&&nextDepthFrame.timeStamp<=timeStamp)
			{
			newDepth=true;
			currentDepthFrame=nextDepthFrame;
			currentMesh=nextMesh;
			nextDepthFrame=depthFrames[(mostRecentDepthFrame-numDepthFrames+numQueueSlots+1)%numQueueSlots];
			nextMesh=meshes[(mostRecentDepthFrame-numDepthFrames+numQueueSlots+1)%numQueueSlots];
			if(--numDepthFrames==numQueueSlots-1)
				depthFrameQueueFullCond.broadcast();
			}
		}
	
	/* Update the projector: */
	if(newColor&&currentColorFrame.isValid())
		projector->setColorFrame(currentColorFrame);
	#if KINECT_CONFIG_USE_SHADERPROJECTOR
	if(newDepth&&currentDepthFrame.isValid())
		projector->setDepthFrame(currentDepthFrame);
	#elif KINECT_CONFIG_USE_PROJECTOR2
	if(newDepth&&currentDepthFrame.isValid()&&currentMesh.isValid())
		projector->setMesh(currentDepthFrame,currentMesh);
	#else
	if(newDepth&&currentMesh.isValid())
		projector->setMesh(currentMesh);
	#endif
	projector->updateFrames();
	}

/******************************************************
Static elements of class KinectViewer::PauseViewerTool:
******************************************************/

KinectViewer::PauseViewerToolFactory* KinectViewer::PauseViewerTool::factory=0;

/**********************************************
Methods of class KinectViewer::PauseViewerTool:
**********************************************/

void KinectViewer::PauseViewerTool::initClass(void)
	{
	PauseViewerToolFactory* pauseViewerToolFactory=new PauseViewerToolFactory("KinectViewerPauseViewerTool","Pause KinectViewer",0,*Vrui::getToolManager());
	pauseViewerToolFactory->setNumButtons(1);
	pauseViewerToolFactory->setButtonFunction(0,"Pause/Unpause");
	Vrui::getToolManager()->addClass(pauseViewerToolFactory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

KinectViewer::PauseViewerTool::PauseViewerTool(const Vrui::ToolFactory* sFactory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(sFactory,inputAssignment)
	{
	}

const Vrui::ToolFactory* KinectViewer::PauseViewerTool::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::PauseViewerTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(vislet==0)
		return;
	
	if(cbData->newButtonState)
		{
		/* Toggle the pause state of all live renderers in the associated vislet: */
		for(std::vector<Renderer*>::iterator rIt=vislet->renderers.begin();rIt!=vislet->renderers.end();++rIt)
			{
			LiveRenderer* lr=dynamic_cast<LiveRenderer*>(*rIt);
			if(lr!=0)
				lr->paused=!lr->paused;
			}
		}
	}

/*****************************************************
Static elements of class KinectViewer::MapTextureTool:
*****************************************************/

KinectViewer::MapTextureToolFactory* KinectViewer::MapTextureTool::factory=0;

/*********************************************
Methods of class KinectViewer::MapTextureTool:
*********************************************/

void KinectViewer::MapTextureTool::initClass(void)
	{
	MapTextureToolFactory* mapTextureToolFactory=new MapTextureToolFactory("KinectViewerMapTextureTool","Toggle KinectViewer Texture",0,*Vrui::getToolManager());
	mapTextureToolFactory->setNumButtons(1);
	mapTextureToolFactory->setButtonFunction(0,"Toggle Texture Mapping");
	Vrui::getToolManager()->addClass(mapTextureToolFactory,Vrui::ToolManager::defaultToolFactoryDestructor);
	}

KinectViewer::MapTextureTool::MapTextureTool(const Vrui::ToolFactory* sFactory,const Vrui::ToolInputAssignment& inputAssignment)
	:Vrui::Tool(sFactory,inputAssignment),
	 mapTexture(false)
	{
	}

const Vrui::ToolFactory* KinectViewer::MapTextureTool::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::MapTextureTool::buttonCallback(int buttonSlotIndex,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	if(vislet==0)
		return;
	
	if(cbData->newButtonState)
		{
		mapTexture=!mapTexture;
		
		#if KINECT_CONFIG_USE_PROJECTOR2
		
		/* Toggle the pause state of all live renderers in the associated vislet: */
		for(std::vector<Renderer*>::iterator rIt=vislet->renderers.begin();rIt!=vislet->renderers.end();++rIt)
			(*rIt)->getProjector().setMapTexture(mapTexture);
		
		#endif
		}
	}

/*************************************
Static elements of class KinectViewer:
*************************************/

KinectViewerFactory* KinectViewer::factory=0;

/*****************************
Methods of class KinectViewer:
*****************************/

void KinectViewer::toolCreationCallback(Vrui::ToolManager::ToolCreationCallbackData* cbData)
	{
	/* Check if the new tool is a filming tool: */
	Tool* tool=dynamic_cast<Tool*>(cbData->tool);
	if(tool!=0)
		{
		/* Associate the tool with this vislet: */
		tool->setVislet(this);
		}
	}

KinectViewer::KinectViewer(int numArguments,const char* const arguments[])
	:navigational(false),
	 synched(false),
	 startDisabled(false),firstEnable(true),enabled(false)
	{
	/* Parse the command line: */
	bool highres=false;
	bool compressDepth=false;
	bool applyPreTransform=false;
	Kinect::FrameSource::ExtrinsicParameters preTransform;
	Vrui::InputDevice* cameraTrackingDevice=0;
	const char* saveFileNameBase=0;
	unsigned int saveFileIndex=0;
	for(int i=0;i<numArguments;++i)
		{
		if(strcasecmp(arguments[i],"-navigational")==0||strcasecmp(arguments[i],"-n")==0)
			navigational=true;
		else if(strcasecmp(arguments[i],"-high")==0)
			highres=true;
		else if(strcasecmp(arguments[i],"-low")==0)
			highres=false;
		else if(strcasecmp(arguments[i],"-compress")==0)
			compressDepth=true;
		else if(strcasecmp(arguments[i],"-nocompress")==0)
			compressDepth=false;
		else if(strcasecmp(arguments[i],"-preTransform")==0)
			{
			++i;
			if(i<numArguments)
				{
				try
					{
					preTransform=Misc::ValueCoder<Kinect::FrameSource::ExtrinsicParameters>::decode(arguments[i],arguments[i]+strlen(arguments[i]),0);
					applyPreTransform=true;
					}
				catch(Misc::DecodingError err)
					{
					std::cerr<<"KinectViewer: Ignoring malformed pre-transformation due to exception "<<err.what()<<std::endl;
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -preTransform argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-trackDevice")==0)
			{
			++i;
			if(i<numArguments)
				{
				cameraTrackingDevice=Vrui::findInputDevice(arguments[i]);
				if(cameraTrackingDevice==0)
					std::cerr<<"KinectViewer: Tracking input device "<<arguments[i]<<" not found"<<std::endl;
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -trackDevice argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-save")==0)
			{
			++i;
			if(i<numArguments)
				saveFileNameBase=arguments[i];
			else
				std::cerr<<"KinectViewer: Ignoring dangling -save argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-c")==0)
			{
			++i;
			if(i<numArguments)
				{
				/* Connect to a local Kinect device: */
				if(Vrui::getClusterMultiplexer()==0)
					{
					/* Open the camera of the given index: */
					int cameraIndex=atoi(arguments[i]);
					Kinect::Camera* camera=new Kinect::Camera(cameraIndex);
					
					/* Set the camera's frame size and compression flag: */
					camera->setFrameSize(Kinect::FrameSource::COLOR,highres?Kinect::Camera::FS_1280_1024:Kinect::Camera::FS_640_480);
					camera->setCompressDepthFrames(compressDepth);
					
					/* Load the camera's default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a renderer for the camera: */
					LiveRenderer* newRenderer=0;
					if(cameraTrackingDevice!=0)
						{
						/* Create a tracked renderer: */
						newRenderer=new TrackedRenderer(camera,cameraTrackingDevice);
						
						/* Only use the tracker for this camera: */
						cameraTrackingDevice=0;
						}
					else
						newRenderer=new LiveRenderer(camera);
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"KinectViewer: Ignoring -c "<<arguments[i]<<" argument: Streaming from local Kinect camera(s) is not supported in cluster environments"<<std::endl;
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -c argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-c2")==0)
			{
			++i;
			if(i<numArguments)
				{
				/* Connect to a local Kinect V2 device: */
				if(Vrui::getClusterMultiplexer()==0)
					{
					/* Open the camera of the given index: */
					int cameraIndex=atoi(arguments[i]);
					Kinect::CameraV2* camera=new Kinect::CameraV2(cameraIndex);
					
					// DEBUGGING
					std::cout<<"Created Kinect V2 camera for index "<<cameraIndex<<std::endl;
					
					/* Load the camera's default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a renderer for the camera: */
					LiveRenderer* newRenderer=0;
					if(cameraTrackingDevice!=0)
						{
						/* Create a tracked renderer: */
						newRenderer=new TrackedRenderer(camera,cameraTrackingDevice);
						
						/* Only use the tracker for this camera: */
						cameraTrackingDevice=0;
						}
					else
						newRenderer=new LiveRenderer(camera);
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"KinectViewer: Ignoring -c2 "<<arguments[i]<<" argument: Streaming from local Kinect camera(s) is not supported in cluster environments"<<std::endl;
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -c2 argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-rs")==0)
			{
			++i;
			if(i<numArguments)
				{
				#if KINECT_CONFIG_HAVE_LIBREALSENSE
				
				/* Connect to a local Intel RealSense device: */
				if(Vrui::getClusterMultiplexer()==0)
					{
					/* Open the RealSense camera of the given index: */
					int cameraIndex=atoi(arguments[i]);
					Kinect::CameraRealSense* camera=new Kinect::CameraRealSense(cameraIndex);
					
					/* Load the camera's default background image: */
					if(camera->loadDefaultBackground())
						camera->setRemoveBackground(true);
					
					/* Add a renderer for the camera: */
					LiveRenderer* newRenderer=0;
					if(cameraTrackingDevice!=0)
						{
						/* Create a tracked renderer: */
						newRenderer=new TrackedRenderer(camera,cameraTrackingDevice);
						
						/* Only use the tracker for this camera: */
						cameraTrackingDevice=0;
						}
					else
						newRenderer=new LiveRenderer(camera);
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				else if(Vrui::isMaster())
					{
					/* Can't stream from local camera in cluster mode: */
					std::cerr<<"KinectViewer: Ignoring -rs "<<arguments[i]<<" argument: Streaming from local Intel RealSense camera(s) is not supported in cluster environments"<<std::endl;
					}
				
				#else
				
				std::cerr<<"KinectViewer: Intel RealSense cameras not supported on host"<<std::endl;
				
				#endif
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -rs argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-f")==0)
			{
			/* Open a 3D video stream file: */
			++i;
			if(i<numArguments)
				{
				/* Add a synchronized renderer for the given video file: */
				renderers.push_back(new SynchedRenderer(arguments[i]));
				
				synched=true;
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -f argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-p")==0)
			{
			/* Connect to a 3D video streaming server: */
			i+=2;
			if(i<numArguments)
				{
				Kinect::MultiplexedFrameSource* source=Kinect::MultiplexedFrameSource::create(Cluster::openTCPPipe(Vrui::getClusterMultiplexer(),arguments[i-1],atoi(arguments[i])));
				
				/* Add a renderer for each component stream in the multiplexer: */
				for(unsigned int i=0;i<source->getNumStreams();++i)
					{
					LiveRenderer* newRenderer=new LiveRenderer(source->getStream(i));
					renderers.push_back(newRenderer);
					
					/* Check if the camera's stream needs to be saved: */
					if(saveFileNameBase!=0)
						{
						/* Construct the save file name: */
						std::string saveFileName=saveFileNameBase;
						char index[10];
						saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
						newRenderer->saveStreams(saveFileName);
						++saveFileIndex;
						
						synched=true;
						}
					}
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -p argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-cfg")==0)
			{
			/* Create a renderer based on a configuration file section: */
			++i;
			if(i<numArguments)
				{
				std::cout<<"KinectViewer: Loading renderer from configuration file section "<<arguments[i]<<std::endl;
				
				/* Go to the requested configuration file section: */
				Misc::ConfigurationFileSection cfg=Vrui::getVisletManager()->getVisletClassSection("KinectViewer").getSection(arguments[i]);
				
				/* Query the camera type: */
				std::string cameraType=cfg.retrieveString("./cameraType");
				unsigned int cameraIndex=cfg.retrieveValue<unsigned int>("./cameraIndex",0U);
				Kinect::DirectFrameSource* source=0;
				if(cameraType=="Kinect")
					source=new Kinect::Camera(cameraIndex);
				else if(cameraType=="KinectV2")
					source=new Kinect::CameraV2(cameraIndex);
				#if KINECT_CONFIG_HAVE_LIBREALSENSE
				else if(cameraType=="RealSense")
					source=new Kinect::CameraRealSense(cameraIndex);
				#endif
				else
					Misc::throwStdErr("KinectViewer: Unsupported camera type %s",cameraType.c_str());
				
				std::cout<<"Loaded camera of type "<<cameraType<<std::endl;
				
				/* Try loading the sources's default background image: */
				if(source->loadDefaultBackground())
					source->setRemoveBackground(true);
				
				/* Configure the source from the configuration file section: */
				source->configure(cfg);
				
				/* Create a renderer: */
				LiveRenderer* renderer=0;
				if(cfg.hasTag("./trackingDevice"))
					{
					/* Create a tracked renderer: */
					TrackedRenderer* tr=new TrackedRenderer(source,Vrui::findInputDevice(cfg.retrieveString("./trackingDevice").c_str()));
					
					if(cfg.hasTag("./latency"))
						tr->latency=cfg.retrieveValue<double>("./latency");
					renderer=tr;
					}
				else
					{
					/* Create a live renderer: */
					renderer=new LiveRenderer(source);
					}
				
				/* Check if the renderer has a pre-transformation: */
				if(cfg.hasTag("./preTransform"))
					renderer->applyPreTransform(cfg.retrieveValue<Kinect::FrameSource::ExtrinsicParameters>("./preTransform"));
				
				/* Configure the renderer's projector: */
				if(cfg.hasTag("./triangleDepthRange"))
					renderer->getProjector().setTriangleDepthRange(cfg.retrieveValue<Kinect::FrameSource::DepthPixel>("./triangleDepthRange"));
				
				#if KINECT_CONFIG_USE_PROJECTOR2
				if(cfg.hasTag("./mapTexture"))
					renderer->getProjector().setMapTexture(cfg.retrieveValue<bool>("./mapTexture"));
				#endif
				
				/* Check if the camera's stream needs to be saved: */
				if(saveFileNameBase!=0)
					{
					/* Construct the save file name: */
					std::string saveFileName=saveFileNameBase;
					char index[10];
					saveFileName.append(Misc::print(saveFileIndex,index+sizeof(index)-1));
					renderer->saveStreams(saveFileName);
					++saveFileIndex;
					
					synched=true;
					}
				
				renderers.push_back(renderer);
				}
			else
				std::cerr<<"KinectViewer: Ignoring dangling -cfg argument"<<std::endl;
			}
		else if(strcasecmp(arguments[i],"-startDisabled")==0||strcasecmp(arguments[i],"-sd")==0)
			startDisabled=true;
		}
	
	if(applyPreTransform)
		{
		/* Apply the pre-transformation to all Kinect projectors: */
		for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
			(*rIt)->applyPreTransform(preTransform);
		}
	
	/* Install callbacks with the tool manager: */
	Vrui::getToolManager()->getToolCreationCallbacks().add(this,&KinectViewer::toolCreationCallback);
	}

KinectViewer::~KinectViewer(void)
	{
	/* Uninstall tool manager callbacks: */
	Vrui::getToolManager()->getToolCreationCallbacks().remove(this,&KinectViewer::toolCreationCallback);
	
	/* Delete all renderers: */
	for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		delete *rIt;
	}

Vrui::VisletFactory* KinectViewer::getFactory(void) const
	{
	return factory;
	}

void KinectViewer::disable(void)
	{
	/* Don't call the base class method when saving 3D video; we still need to receive frame updates even when invisible: */
	if(!synched)
		Vislet::disable();
	
	enabled=false;
	}

void KinectViewer::enable(void)
	{
	/* Call the base class method: */
	Vislet::enable();
	
	if(firstEnable)
		{
		/* Establish a time base: */
		timeBase.set();
		
		/* Start streaming on all renderers: */
		for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
			(*rIt)->startStreaming(timeBase);
		
		enabled=!startDisabled;
		
		firstEnable=false;
		}
	else
		enabled=true;
	}

void KinectViewer::frame(void)
	{
	/* Update all renderers: */
	for(std::vector<Renderer*>::iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->frame(Vrui::getApplicationTime());
	}

void KinectViewer::display(GLContextData& contextData) const
	{
	if(!enabled)
		return;
	
	if(navigational)
		{
		/* Go to navigational coordinates: */
		glPushMatrix();
		glLoadMatrix(Vrui::getDisplayState(contextData).modelviewNavigational);
		}
	
	/* Draw the current 3D video facades of all renderers: */
	for(std::vector<Renderer*>::const_iterator rIt=renderers.begin();rIt!=renderers.end();++rIt)
		(*rIt)->glRenderAction(contextData);
	
	if(navigational)
		{
		/* Go back to physical coordinates: */
		glPopMatrix();
		}
	}
