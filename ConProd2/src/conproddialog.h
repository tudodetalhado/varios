#ifndef CONPRODDIALOG_H
#define CONPRODDIALOG_H

#include <QDialog>
#include "consumer.h"
#include "producer.h"

#include <QSemaphore>


namespace Ui {
class ConProdDialog;
}

class ConProdDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConProdDialog(QWidget *parent = 0);
    ~ConProdDialog();

public slots:
    void onBufferValueChanged(int);
    void onProducerValueChanged(int);
    void onConsumerValueChanged(int);

private slots:
    void on_startButton_clicked();

private:
    Ui::ConProdDialog *ui;
    Producer *mProducer;
    Consumer *mConsumer;
};

#endif // CONPRODDIALOG_H
