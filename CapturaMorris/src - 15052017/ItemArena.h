#pragma once
#ifndef ARENAWIEW_H
#define ARENAWIEW_H

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>

class ItemArena : public QGraphicsItem
{
public:
    ItemArena();
    //QRectF visibleRect();
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    bool pressionado;

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    //void resizeEvent(QResizeEvent *);
    //void showEvent(QShowEvent *);
    //void fitView();
    //void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    //void mousePressEvent(QMouseEvent* event);
    //void mouseMoveEvent(QMouseEvent* event);

};

#endif // ARENAWIEW_H
