#include "CameraConnectDialog.h"
#include "ui_CameraConnectDialog.h"
// Qt
#include <QtCore/QThread>
#include <QMessageBox>

CameraConnectDialog::CameraConnectDialog(QWidget *parent, bool isStreamSyncEnabled) :
    QDialog(parent),
    ui(new Ui::CameraConnectDialog)
{  
    // Setup dialog
    ui->setupUi(this);

    // deviceNumberEdit (device number) input validation
    QRegExp rx1("^[0-9]{1,3}$"); // Integers 0 to 999
    QRegExpValidator *validator1 = new QRegExpValidator(rx1, 0);
    ui->deviceNumberEdit->setValidator(validator1);

    // imageBufferSizeEdit (image buffer size) input validation
    QRegExp rx2("^[0-9]{1,3}$"); // Integers 0 to 999
    QRegExpValidator *validator2 = new QRegExpValidator(rx2, 0);
    ui->imageBufferSizeEdit->setValidator(validator2);

    // resWEdit (resolution: width) input validation
    QRegExp rx3("^[0-9]{1,4}$"); // Integers 0 to 9999
    QRegExpValidator *validator3 = new QRegExpValidator(rx3, 0);
    ui->resWEdit->setValidator(validator3);

    // resHEdit (resolution: height) input validation
    QRegExp rx4("^[0-9]{1,4}$"); // Integers 0 to 9999
    QRegExpValidator *validator4 = new QRegExpValidator(rx4, 0);
    ui->resHEdit->setValidator(validator4);

    // Setup combo boxes
    QStringList threadPriorities;
    threadPriorities<<"Idle"<<"Lowest"<<"Low"<<"Normal"<<"High"<<"Highest"<<"Time Critical"<<"Inherit";
    ui->capturePrioComboBox->addItems(threadPriorities);
    ui->processingPrioComboBox->addItems(threadPriorities);

    // Set dialog to defaults
    resetToDefaults();

    // Enable/disable checkbox
    ui->enableFrameProcessingCheckBox->setEnabled(isStreamSyncEnabled);

    // Connect button to slot
    connect(ui->resetToDefaultsPushButton,SIGNAL(released()),SLOT(resetToDefaults()));
}

CameraConnectDialog::~CameraConnectDialog()
{
    delete ui;
}

int CameraConnectDialog::getDeviceNumber()
{
    // Set device number to default (any available camera) if field is blank
    if(ui->deviceNumberEdit->text().isEmpty())
    {
        QMessageBox::warning(this->parentWidget(),
                             "WARNING:","Device Number field blank.\nAutomatically set to 0.");
        return 0;
    }
    else
        return ui->deviceNumberEdit->text().toInt();
}

int CameraConnectDialog::getResolutionWidth()
{
    // Return -1 if field is blank
    if(ui->resWEdit->text().isEmpty())
        return -1;
    else
        return ui->resWEdit->text().toInt();
}

int CameraConnectDialog::getResolutionHeight()
{
    // Return -1 if field is blank
    if(ui->resHEdit->text().isEmpty())
        return -1;
    else
        return ui->resHEdit->text().toInt();
}

int CameraConnectDialog::getImageBufferSize()
{
    // Set image buffer size to default if field is blank
    if(ui->imageBufferSizeEdit->text().isEmpty())
    {
        QMessageBox::warning(this->parentWidget(),
            "WARNING:","Image Buffer Size field blank.\nAutomatically set to default value.");
        return DEFAULT_IMAGE_BUFFER_SIZE;
    }
    // Set image buffer size to default if field is zero
    else if(ui->imageBufferSizeEdit->text().toInt()==0)
    {
        QMessageBox::warning(this->parentWidget(),
            "WARNING:","Image Buffer Size cannot be zero.\nAutomatically set to default value.");
        return DEFAULT_IMAGE_BUFFER_SIZE;;
    }
    // Use image buffer size specified by user
    else
        return ui->imageBufferSizeEdit->text().toInt();
}

bool CameraConnectDialog::getDropFrameCheckBoxState()
{
    return ui->dropFrameCheckBox->isChecked();
}

int CameraConnectDialog::getCaptureThreadPrio()
{
    return ui->capturePrioComboBox->currentIndex();
}

int CameraConnectDialog::getProcessingThreadPrio()
{
    return ui->processingPrioComboBox->currentIndex();
}

QString CameraConnectDialog::getTabLabel()
{
    return ui->tabLabelEdit->text();
}

bool CameraConnectDialog::getEnableFrameProcessingCheckBoxState()
{
    return ui->enableFrameProcessingCheckBox->isChecked();
}

void CameraConnectDialog::resetToDefaults()
{
    // Default camera
    ui->deviceNumberEdit->clear();
    // Resolution
    ui->resWEdit->clear();
    ui->resHEdit->clear();
    // Image buffer size
    ui->imageBufferSizeEdit->setText(QString::number(DEFAULT_IMAGE_BUFFER_SIZE));
    // Drop frames
    ui->dropFrameCheckBox->setChecked(DEFAULT_DROP_FRAMES);
    // Capture thread
    if(DEFAULT_CAP_THREAD_PRIO==QThread::IdlePriority)
        ui->capturePrioComboBox->setCurrentIndex(0);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::LowestPriority)
        ui->capturePrioComboBox->setCurrentIndex(1);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::LowPriority)
        ui->capturePrioComboBox->setCurrentIndex(2);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::NormalPriority)
        ui->capturePrioComboBox->setCurrentIndex(3);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::HighPriority)
        ui->capturePrioComboBox->setCurrentIndex(4);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::HighestPriority)
        ui->capturePrioComboBox->setCurrentIndex(5);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::TimeCriticalPriority)
        ui->capturePrioComboBox->setCurrentIndex(6);
    else if(DEFAULT_CAP_THREAD_PRIO==QThread::InheritPriority)
        ui->capturePrioComboBox->setCurrentIndex(7);
    // Processing thread
    if(DEFAULT_PROC_THREAD_PRIO==QThread::IdlePriority)
        ui->processingPrioComboBox->setCurrentIndex(0);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::LowestPriority)
        ui->processingPrioComboBox->setCurrentIndex(1);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::LowPriority)
        ui->processingPrioComboBox->setCurrentIndex(2);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::NormalPriority)
        ui->processingPrioComboBox->setCurrentIndex(3);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::HighPriority)
        ui->processingPrioComboBox->setCurrentIndex(4);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::HighestPriority)
        ui->processingPrioComboBox->setCurrentIndex(5);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::TimeCriticalPriority)
        ui->processingPrioComboBox->setCurrentIndex(6);
    else if(DEFAULT_PROC_THREAD_PRIO==QThread::InheritPriority)
        ui->processingPrioComboBox->setCurrentIndex(7);
    // Tab label
    ui->tabLabelEdit->setText("");
    // Enable Frame Processing checkbox
    ui->enableFrameProcessingCheckBox->setChecked(true);
}
