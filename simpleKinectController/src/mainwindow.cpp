#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->kinectHeightSlider,SIGNAL(valueChanged(int)),this,SLOT(setNewValue(int)));
    ui->comboBoxKinect->setCurrentIndex(-1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setNewValue(int val){
    this->ui->labelNewHeight->setText(QString::number(val));
}

void MainWindow::setDropDownList(QMap<int, QString> map){
    QMapIterator<int,QString>i(map);
    while(i.hasNext()){
        i.next();
        ui->comboBoxKinect->clear();
        ui->comboBoxKinect->insertItem(i.key(),i.value());
    }
}

void MainWindow::setComboBox(int i){
    ui->comboBoxKinect->setCurrentIndex(i);
}

void MainWindow::displayError(QString error){
    ui->labelError->setText("<font color='red'>" + error + "</font>");
}
