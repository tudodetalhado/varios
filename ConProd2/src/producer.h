#ifndef PRODUCER_H
#define PRODUCER_H

#include <QThread>
#include <QTime>

class Producer : public QThread
{
    Q_OBJECT
public:
    explicit Producer(QObject *parent = 0);
    void run();

signals:
    void bufferFillCountChanged(int bCount);
    void producerCountChanged(int count);

public slots:
};

#endif // PRODUCER_H
