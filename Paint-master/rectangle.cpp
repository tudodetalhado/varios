/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "shape.h"

namespace Paint {

class Rectangle : public Shape
{
public:
    explicit Rectangle(const QPoint &topLeft,
                       int penWidth,
                       const QColor& penColor);

protected:
    virtual void doDraw(QPainter &painter) override;

    virtual QRect doRect() const override;

    virtual void doUpdate(const QPoint &toPoint) override;


private:
    QRect r;
};

Rectangle::Rectangle(const QPoint &topLeft,
                     int penWidth,
                     const QColor &penColor) :
    Shape(penWidth, penColor), r(topLeft, topLeft)
{
}

void Rectangle::doDraw(QPainter &painter)
{
    if (!r.isNull()) {
        painter.drawRect(r.normalized());
    }
}

QRect Rectangle::doRect() const
{
    return r.normalized();
}

void Rectangle::doUpdate(const QPoint &toPoint)
{
    r.setBottomRight(toPoint);
}

std::unique_ptr<Shape> createRectangle(const QPoint &topLeft,
                                       int penWidth,
                                       const QColor& penColor)
{
    return std::unique_ptr<Shape>(new Rectangle(topLeft, penWidth, penColor));
}

} // namespace Paint
