#include <QApplication>

#include "infinitecountworker.h"
#include "portablesleep.h"

KinectTrack::KinectTrack()
    : m_running(true)
{
}

void KinectTrack::doWork()
{
    int i = 0;
    while (m_running) {
        emit updateCount(i);
        i++;
        contador++;
        qDebug() << "Contador: " << contador;
        qApp->processEvents();
        PortableSleep::msleep(1000);
    }
    emit finished();
}

void KinectTrack::stopWork()
{
    contador = 999;
    qDebug() << "Contador: " << contador;
    m_running = false;
}
