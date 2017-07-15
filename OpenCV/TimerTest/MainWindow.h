#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QMessageBox>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void atualizarDisplay();
    void on_btnStart_clicked();
    void on_btnStop_clicked();

    void on_btnReiniciar_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    double tempo;
};

#endif // MAINWINDOW_H
