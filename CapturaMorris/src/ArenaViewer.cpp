#include "ArenaViewer.h"

ArenaViewer::ArenaViewer(QWidget *parent)
    : QLabel(parent)
{
}


void ArenaViewer::mouseDoubleClickEvent(QMouseEvent *e)
{
//    int j = ButtonTester::buttonByNumber (e->button());
//    QString result = "Mouse DoubleClick: raw button=" + QString::number(j)
//                + "  Qt=" + enumNameFromValue(e->button());
//    QString buttonsString = ButtonTester::enumNamesFromMouseButtons(e->buttons());
//    result += "\n heldbuttons" + buttonsString;
//    qDebug() << result;
//    this->setText(result);
    //int tmp = 0;
    //tmp = 1;
    emit cliqueDuplo();
}
