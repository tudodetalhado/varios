#pragma once
#ifndef ARENAVIEW_H
#define ARENAVIEW_H

#include <QLabel>

class LabelViewer : public QLabel
{
    Q_OBJECT
public:
    //LabelViewer();
    explicit LabelViewer(QWidget *parent = 0);
    //void    mousePressEvent(QMouseEvent *event) override;
    //void    mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

signals:
    void cliqueDuplo();

};

#endif // ARENAVIEW_H
