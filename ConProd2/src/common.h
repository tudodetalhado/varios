#ifndef COMMON_H
#define COMMON_H

#include <QSemaphore>
#include "myConstants.h"

extern char buffer[BufferSize];

extern QSemaphore freeBytes;
extern QSemaphore usedBytes;

#endif // COMMON_H
