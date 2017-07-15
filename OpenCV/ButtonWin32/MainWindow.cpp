#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <Windows.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QString qstr1 = "Z:";
    QString qstr2 = getenv("tmp");
    DefineDosDevice(0, (LPCTSTR)qstr1.utf16(), (LPCTSTR)qstr2.utf16());
}

void MainWindow::on_pushButton_2_clicked()
{
    QString qstr = "Z:";
    DefineDosDevice(2, (LPCTSTR)qstr.utf16(), 0);
}
