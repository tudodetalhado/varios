/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "shape.h"

#include <QPolygon>

namespace Paint {

class Scribble : public Shape
{
public:
    explicit Scribble(const QPoint &topLeft,
                      int penWidth,
                      const QColor& penColor);

protected:
    virtual void doDraw(QPainter &painter) override;

    virtual QRect doRect() const override;

    virtual void doUpdate(const QPoint &toPoint) override;

private:
    QPolygon poly;
};

Scribble::Scribble(const QPoint &topLeft,
                   int penWidth,
                   const QColor &penColor) :
    Shape(penWidth, penColor)
{
    update(topLeft);
}

void Scribble::doDraw(QPainter &painter)
{
    painter.drawPolyline(poly);
}

QRect Scribble::doRect() const
{
    return poly.boundingRect();
}

void Scribble::doUpdate(const QPoint &toPoint)
{
    poly << toPoint;
}

std::unique_ptr<Shape> createEraser(const QPoint &topLeft,
                                    int penWidth,
                                    const QColor&)
{
    return std::unique_ptr<Shape>(
                new Scribble(topLeft, penWidth, Qt::white));
}

std::unique_ptr<Shape> createScribble(const QPoint &topLeft,
                                      int penWidth,
                                      const QColor& penColor)
{
    return std::unique_ptr<Shape>(new Scribble(topLeft, penWidth, penColor));
}

} // namespace Paint
