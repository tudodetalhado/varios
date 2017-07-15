#ifndef __INFINITE_COUNT_WORKER_H__
#define __INFINITE_COUNT_WORKER_H__

#include <QObject>
#include <QDebug>

class KinectTrack : public QObject
{
    Q_OBJECT

    public:
        KinectTrack();

    public slots:
        void doWork();
        void stopWork();

    signals:
        void updateCount(int);
        void finished();

    private:
        bool m_running;
        int contador = 0;
};

#endif // __INFINITE_COUNT_WORKER_H__
