#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_


#include <QMainWindow>

class AboutDialog;

namespace Ui 
{
	class MainWindow;
}


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	
	explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
	void fileOpen();
	void fileSave();
	void fileSaveAs();
	void aboutDialogShow();

private:

	Ui::MainWindow *ui;
	QString currentFileName;
	AboutDialog* aboutDialog;
};

#endif // _MAIN_WINDOW_H_