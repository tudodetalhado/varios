#ifndef CONSUMER_H
#define CONSUMER_H

#include <QThread>
#include <QTime>


class Consumer : public QThread
{
    Q_OBJECT
public:
    explicit Consumer(QObject *parent = 0);
    void run();

signals:
    //void stringConsumed(const QString &text);
    void bufferFillCountChanged(int cCount);
    void consumerCountChanged(int count);
public slots:
};

#endif // CONSUMER_H
