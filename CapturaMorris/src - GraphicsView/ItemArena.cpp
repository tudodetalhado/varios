#include "ItemArena.h"
#include <QShowEvent>
#include <QResizeEvent>
#include <QPainter>
#include <QPainterPath>

ItemArena::ItemArena()
{
   pressionado = false;
   setFlag(QGraphicsItem::ItemIsMovable);
}

//QRectF ArenaWiew::visibleRect() {
//    QPointF scrollBar(horizontalScrollBar()->value(), verticalScrollBar()->value());
//    QPointF bottomRight = scrollBar + viewport()->rect().bottomRight();
//    QMatrix matrix = matrix().inverted();
//    return matrix.mapRect(QRectF(scrollBar,bottomRight));
//}

QRectF ItemArena::boundingRect() const
{
    int arenaViewWidth  = 600;
    int arenaViewHeight = 500;
    int elipseWidth    = 200;
    int elipseHeight   = 200;
    int left = (arenaViewWidth  - elipseWidth)  / 2;
    int top  = (arenaViewHeight - elipseHeight) / 2;

    //left, top, width, height
    return QRectF(left, top, elipseWidth, elipseHeight);
}

void ItemArena::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    //_painter = painter;
    QRectF limite = boundingRect();
    QBrush brush(QColor(79, 106, 25));

    //Qt::DashLine, Qt::SolidLine
    QPen pen(QColor(79, 106, 25), 1, Qt::DashLine, Qt::FlatCap, Qt::MiterJoin);
    //QPen pen(Qt::black);
    pen.setWidth(0.5);
    painter->setPen(pen);

    if(pressionado)
    {
        brush.setColor(Qt::red);
    }
//    else
//    {
//        brush.setColor(Qt::blue);
//    }

//    QPainterPath path;
//    path.addRect(20, 20, 60, 60);

//    path.moveTo(0, 0);
//    path.cubicTo(99, 0,  50, 50,  99, 99);
//    path.cubicTo(0, 99,  50, 50,  0, 0);

//    QPainter painter(this);
//    painter.fillRect(0, 0, 100, 100, Qt::white);
//    painter.setPen(QPen(QColor(79, 106, 25), 1, Qt::SolidLine,
//                      Qt::FlatCap, Qt::MiterJoin));
//    painter.setBrush(QColor(122, 163, 39));

//    painter.drawPath(path);

//    QPainterPath path;
//    path.addRect(20, 20, 60, 60);

//    path.moveTo(0, 0);
//    path.cubicTo(99, 0,  50, 50,  99, 99);
//    path.cubicTo(0, 99,  50, 50,  0, 0);

//    QPainterPath ellipsePath;
//    //ellipsePath.moveTo(80.0, 50.0);
//    ellipsePath.arcTo(0.0, 0.0, 100.0, 100.0, 0.0, 360.0);

//    painter->fillRect(0, 0, 100, 100, Qt::white);
//    painter->setPen(QPen(QColor(79, 106, 25), 1, Qt::SolidLine,
//                         Qt::FlatCap, Qt::MiterJoin));
//    painter->setBrush(QColor(122, 163, 39));

    QPainterPath ellipsePath;
    //ellipsePath.moveTo(80.0, 50.0);
    //ellipsePath.arcTo(20.0, 30.0, 60.0, 40.0, 0.0, 360.0);
    ellipsePath.moveTo(limite.x() + limite.width(), limite.y() + limite.height() / 2);
    ellipsePath.arcTo(limite.x(), limite.y(), limite.width(), limite.height(), 0.0, 360.0);

    //
    //painter->fillPath(ellipsePath, brush);
    painter->drawPath(ellipsePath);

    //painter->fillRect(rect, brush);
    //painter->drawEllipse(rect);
    //painter->drawRect(rect);
}

//QPointF origem;
void ItemArena::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // Store original position.
    //origem = event->pos();

    pressionado = true;
    update();
    QGraphicsItem::mousePressEvent(event);
}

void ItemArena::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    pressionado = false;
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

