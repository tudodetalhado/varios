#include "conproddialog.h"
#include "ui_conproddialog.h"
#include "myConstants.h"

// BufferSize: maximum bytes that can be stored
char buffer[BufferSize];

QSemaphore freeBytes(BufferSize);
QSemaphore usedBytes;

ConProdDialog::ConProdDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConProdDialog)
{
    ui->setupUi(this);

    // progress bar range setup
    ui->producerProgressBar->setRange(0, DataSize);
    ui->consumerProgressBar->setRange(0, DataSize);
    ui->bufferProgressBar->setRange(0, BufferSize);

    // make two threads
    mProducer = new Producer(this);
    mConsumer = new Consumer(this);

    // connect signal/slot for the buffer progress bar
    connect(mConsumer, SIGNAL(bufferFillCountChanged(int)),
              this, SLOT(onBufferValueChanged(int)));
    connect(mProducer, SIGNAL(bufferFillCountChanged(int)),
              this, SLOT(onBufferValueChanged(int)));

    // connect signal/slot for consumer/producer progress bar
    connect(mConsumer, SIGNAL(consumerCountChanged(int)),
              this, SLOT(onConsumerValueChanged(int)));
    connect(mProducer, SIGNAL(producerCountChanged(int)),
              this, SLOT(onProducerValueChanged(int)));
}

ConProdDialog::~ConProdDialog()
{
    delete ui;
}

void ConProdDialog::onBufferValueChanged(int bCount)
{
    ui->bufferProgressBar->setValue(bCount);
}

void ConProdDialog::onProducerValueChanged(int pCount)
{
    ui->producerProgressBar->setValue(pCount);
}

void ConProdDialog::onConsumerValueChanged(int cCount)
{
    ui->consumerProgressBar->setValue(cCount);
}

// start button clicked
void ConProdDialog::on_startButton_clicked()
{
    // disable the start button
    ui->startButton->setEnabled(false);

    // threads starat
    mProducer->start();
    mConsumer->start();
}

