#ifndef _ABOUT_DIALOG_H_
#define _ABOUT_DIALOG_H_


#include <QDialog>

namespace Ui
{
	class AboutDialog;
}


class AboutDialog : public QDialog
{
	Q_OBJECT

public:
	
	explicit AboutDialog(QWidget *parent = 0);
	~AboutDialog();


private:

	Ui::AboutDialog *ui;

};

#endif // _ABOUT_DIALOG_H_