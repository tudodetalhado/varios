#include "ProcessingThread.h"

ProcessingThread::ProcessingThread(SharedImageBuffer *sharedImageBuffer,
                                   int deviceNumber)
    : QThread(), sharedImageBuffer(sharedImageBuffer)
{
    // Save Device Number
    this->deviceNumber=deviceNumber;

    // Initialize members
    doStop=false;
    sampleNumber=0;
    fpsSum=0;
    fps.clear();
    statsData.averageFPS=0;
    statsData.nFramesProcessed=0;
}

void ProcessingThread::run()
{
    while(1)
    {
        // Stop thread if doStop=TRUE //
        doStopMutex.lock();
        if(doStop)
        {
            doStop=false;
            doStopMutex.unlock();
            break;
        }
        doStopMutex.unlock();

        // Save processing time
        processingTime=t.elapsed();
        // Start timer (used to calculate processing rate)
        t.start();

        processingMutex.lock();
        // Get frame from queue, store in currentFrame, set ROI
        currentFrame=Mat(sharedImageBuffer->getByDeviceNumber(deviceNumber)->get().clone(), currentROI);

        // Example of how to grab a frame from another stream (where Device Number=1)
        // Note: This requires stream synchronization to be ENABLED (in the Options menu of MainWindow) and frame processing for the stream you are grabbing FROM to be DISABLED.
        /*
        if(sharedImageBuffer->containsImageBufferForDeviceNumber(1))
        {
            // Grab frame from another stream (connected to camera with Device Number=1)
            Mat frameFromAnotherStream = Mat(sharedImageBuffer->getByDeviceNumber(1)->getFrame(), currentROI);
            // Linear blend images together using OpenCV and save the result to currentFrame. Note: beta=1-alpha
            addWeighted(frameFromAnotherStream, 0.5, currentFrame, 0.5, 0.0, currentFrame);
        }
        */

        ////////////////////////////////////
        // PERFORM IMAGE PROCESSING BELOW //
        ////////////////////////////////////
        // Grayscale conversion (in-place operation)
        if(imgProcFlags.grayscaleOn && (currentFrame.channels() == 3 || currentFrame.channels() == 4))
            cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);

        // Smooth (in-place operations)
        if(imgProcFlags.smoothOn)
        {
            switch(imgProcSettings.smoothType)
            {
                // BLUR
                case 0:
                    blur(currentFrame, currentFrame,
                         Size(imgProcSettings.smoothParam1, imgProcSettings.smoothParam2));
                    break;
                // GAUSSIAN
                case 1:
                    GaussianBlur(currentFrame, currentFrame,
                                 Size(imgProcSettings.smoothParam1, imgProcSettings.smoothParam2),
                                 imgProcSettings.smoothParam3, imgProcSettings.smoothParam4);
                    break;
                // MEDIAN
                case 2:
                    medianBlur(currentFrame, currentFrame,
                               imgProcSettings.smoothParam1);
                    break;
            }
        }
        // Dilate
        if(imgProcFlags.dilateOn)
        {
            dilate(currentFrame, currentFrame,
                   Mat(), Point(-1, -1), imgProcSettings.dilateNumberOfIterations);
        }
        // Erode
        if(imgProcFlags.erodeOn)
        {
            erode(currentFrame, currentFrame,
                  Mat(), Point(-1, -1), imgProcSettings.erodeNumberOfIterations);
        }
        // Flip
        if(imgProcFlags.flipOn)
        {
            flip(currentFrame, currentFrame,
                 imgProcSettings.flipCode);
        }
        // Canny edge detection
        if(imgProcFlags.cannyOn)
        {
            Canny(currentFrame, currentFrame,
                  imgProcSettings.cannyThreshold1, imgProcSettings.cannyThreshold2,
                  imgProcSettings.cannyApertureSize, imgProcSettings.cannyL2gradient);
        }
        ////////////////////////////////////
        // PERFORM IMAGE PROCESSING ABOVE //
        ////////////////////////////////////

        // Convert Mat to QImage
        frame=MatToQImage(currentFrame);
        processingMutex.unlock();

        // Inform GUI thread of new frame (QImage)
        emit newFrame(frame);

        // Update statistics
        updateFPS(processingTime);
        statsData.nFramesProcessed++;
        // Inform GUI of updated statistics
        emit updateStatisticsInGUI(statsData);
    }
    qDebug() << "Stopping processing thread...";
}

void ProcessingThread::updateFPS(int timeElapsed)
{
    // Add instantaneous FPS value to queue
    if(timeElapsed>0)
    {
        fps.enqueue((int)1000/timeElapsed);
        // Increment sample number
        sampleNumber++;
    }

    // Maximum size of queue is DEFAULT_PROCESSING_FPS_STAT_QUEUE_LENGTH
    if(fps.size()>PROCESSING_FPS_STAT_QUEUE_LENGTH)
        fps.dequeue();

    // Update FPS value every DEFAULT_PROCESSING_FPS_STAT_QUEUE_LENGTH samples
    if((fps.size()==PROCESSING_FPS_STAT_QUEUE_LENGTH)&&(sampleNumber==PROCESSING_FPS_STAT_QUEUE_LENGTH))
    {
        // Empty queue and store sum
        while(!fps.empty())
            fpsSum+=fps.dequeue();
        // Calculate average FPS
        statsData.averageFPS=fpsSum/PROCESSING_FPS_STAT_QUEUE_LENGTH;
        // Reset sum
        fpsSum=0;
        // Reset sample number
        sampleNumber=0;
    }
}

void ProcessingThread::stop()
{
    QMutexLocker locker(&doStopMutex);
    doStop=true;
}

void ProcessingThread::updateImageProcessingFlags(struct ImageProcessingFlags imgProcFlags)
{
    QMutexLocker locker(&processingMutex);
    this->imgProcFlags.grayscaleOn=imgProcFlags.grayscaleOn;
    this->imgProcFlags.smoothOn=imgProcFlags.smoothOn;
    this->imgProcFlags.dilateOn=imgProcFlags.dilateOn;
    this->imgProcFlags.erodeOn=imgProcFlags.erodeOn;
    this->imgProcFlags.flipOn=imgProcFlags.flipOn;
    this->imgProcFlags.cannyOn=imgProcFlags.cannyOn;
}

void ProcessingThread::updateImageProcessingSettings(struct ImageProcessingSettings imgProcSettings)
{
    QMutexLocker locker(&processingMutex);
    this->imgProcSettings.smoothType=imgProcSettings.smoothType;
    this->imgProcSettings.smoothParam1=imgProcSettings.smoothParam1;
    this->imgProcSettings.smoothParam2=imgProcSettings.smoothParam2;
    this->imgProcSettings.smoothParam3=imgProcSettings.smoothParam3;
    this->imgProcSettings.smoothParam4=imgProcSettings.smoothParam4;
    this->imgProcSettings.dilateNumberOfIterations=imgProcSettings.dilateNumberOfIterations;
    this->imgProcSettings.erodeNumberOfIterations=imgProcSettings.erodeNumberOfIterations;
    this->imgProcSettings.flipCode=imgProcSettings.flipCode;
    this->imgProcSettings.cannyThreshold1=imgProcSettings.cannyThreshold1;
    this->imgProcSettings.cannyThreshold2=imgProcSettings.cannyThreshold2;
    this->imgProcSettings.cannyApertureSize=imgProcSettings.cannyApertureSize;
    this->imgProcSettings.cannyL2gradient=imgProcSettings.cannyL2gradient;
}

void ProcessingThread::setROI(QRect roi)
{
    QMutexLocker locker(&processingMutex);
    currentROI.x = roi.x();
    currentROI.y = roi.y();
    currentROI.width = roi.width();
    currentROI.height = roi.height();
}

QRect ProcessingThread::getCurrentROI()
{
    return QRect(currentROI.x, currentROI.y, currentROI.width, currentROI.height);
}
