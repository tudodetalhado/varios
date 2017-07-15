#ifndef COMMON_H
#define COMMON_H

#include <QSemaphore>
#include "Constantes.h"
#include "CircularBuffer.h"
#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>

//extern boost::circular_buffer<cv::Mat> _queue;
extern CircularBuffer<cv::Mat> _queue;
extern QSemaphore *freeSlots;
extern QSemaphore *usedSlots;


#endif // COMMON_H
