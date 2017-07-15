#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMap>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    
private:
    Ui::MainWindow *ui;
private slots:
    void setNewValue(int val);
public slots:
    void setDropDownList(QMap<int,QString> map);
    void setComboBox(int i);
    void displayError(QString error);
};

#endif // MAINWINDOW_H
