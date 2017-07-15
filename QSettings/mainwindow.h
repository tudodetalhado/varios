#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
 Q_OBJECT
public:

 explicit MainWindow(QWidget *parent = 0);
 virtual ~MainWindow();

private:

 void initGui();

 void loadSettings();

 void saveSettings();

private slots:

 void handleButton();

private:

 QString m_sSettingsFile;

 QLabel* m_pLabel;

 QLineEdit* m_pEdit;

 QPushButton* m_pButton;
};

#endif // MAINWINDOW_H
