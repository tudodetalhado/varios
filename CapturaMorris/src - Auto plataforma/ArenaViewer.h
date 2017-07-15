#ifndef ARENAVIEW_H
#define ARENAVIEW_H

#include <QLabel>
#include <QWidget>
#include <QMouseEvent>

class ArenaViewer : public QLabel
{
    Q_OBJECT
public:
    explicit ArenaViewer(QWidget *parent = 0);
    //void    mousePressEvent(QMouseEvent *event) override;
    //void    mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

signals:
    void cliqueDuplo();

};

#endif // ARENAVIEW_H
