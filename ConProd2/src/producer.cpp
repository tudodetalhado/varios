#include "producer.h"
#include "common.h"
#include <QDebug>


Producer::Producer(QObject *parent) :
    QThread(parent)
{
}

void Producer::run()
{
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    for (int i = 0; i < DataSize; ++i)
    {
        freeBytes.acquire();
        buffer[i % BufferSize] = "ACGT"[(int)qrand() % 4];
        usedBytes.release();
        if(i % 20 == 0)
        emit bufferFillCountChanged(usedBytes.available());
        emit producerCountChanged(i);
    }
}
