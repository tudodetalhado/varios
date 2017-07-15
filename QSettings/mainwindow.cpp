#include "mainwindow.h"
#include <QCoreApplication>
#include <QSettings>
#include <QApplication>

MainWindow::MainWindow(QWidget *parent)
 : QMainWindow(parent),
 m_pLabel(NULL),
 m_pEdit(NULL),
 m_pButton(NULL)
{
 initGui();

 m_sSettingsFile = QApplication::applicationDirPath().left(1) + ":/demosettings.ini";
 loadSettings();

if (m_pButton)
 {
  connect(m_pButton, SIGNAL (released()),this, SLOT (handleButton()));
 }
}

void MainWindow::initGui()
{
 m_pLabel = new QLabel("", this);
 m_pLabel->setGeometry(0,0, 200,40);
 m_pEdit = new QLineEdit("", this);
 m_pEdit->setGeometry(0,40, 200,40);
 m_pButton = new QPushButton("OK", this);
 m_pButton->setGeometry(0,80, 200,40);
}

void MainWindow::loadSettings()
{
 QSettings settings(m_sSettingsFile, QSettings::NativeFormat);
 QString sText = settings.value("text", "").toString();
 if (m_pLabel)
 {
  m_pLabel->setText(sText);
 }
}

void MainWindow::saveSettings()
{
 QSettings settings(m_sSettingsFile, QSettings::NativeFormat);
 QString sText = (m_pEdit) ? m_pEdit->text() : "";
 settings.setValue("text", sText);
 if (m_pLabel)
 {
  m_pLabel->setText(sText);
 }
}

void MainWindow::handleButton()
{
 saveSettings();
}

MainWindow::~MainWindow()
{

}
