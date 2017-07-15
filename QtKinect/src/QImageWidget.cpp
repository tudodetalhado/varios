

#include <QtWidgets>
#include "QImageWidget.h"
#include <iostream>

QImageWidget::QImageWidget(QWidget* parent) : QLabel(parent)
{
}


void QImageWidget::setImage(const QImage& image)
{
	if (!image.isNull())
	{
		//resize(image.size());
		//setPixmap(QPixmap::fromImage(image));
		setPixmap(QPixmap::fromImage(image).scaled(width(), height(), Qt::KeepAspectRatio));
	}
}


bool QImageWidget::loadFile(const QString &fileName)
{
    QImage image = QImage(fileName);
    if (image.isNull()) 
	{
        QMessageBox::information(this, QGuiApplication::applicationDisplayName(),
                                 tr("Cannot load %1.").arg(QDir::toNativeSeparators(fileName)));
        setWindowFilePath(QString());
        setPixmap(QPixmap());
        adjustSize();
        return false;
    }

	setImage(image);

    setWindowFilePath(fileName);
    return true;
}



//void QImageWidget::paintEvent(QPaintEvent* event)
//{
//	QLabel::paintEvent(event);
//}


void QImageWidget::keyReleaseEvent(QKeyEvent *e)
{
	if (e->key() == Qt::Key_Q || e->key() == Qt::Key_Escape)
		this->close();

	if (e->modifiers() == Qt::ControlModifier && e->key() == Qt::Key_S)
	{
		const QString& filename = QFileDialog::getSaveFileName(this,
			tr("Save Image File"), "../../data/",
			tr("Images (*.png);;Images (*.jpg)"));

		this->save(filename);
	}

}



void QImageWidget::closeEvent(QCloseEvent *event)
{
	QLabel::closeEvent(event);
	emit closed();
}


void QImageWidget::save(const QString& filename)
{
	if (!filename.isEmpty())
		this->pixmap()->save(filename);
}