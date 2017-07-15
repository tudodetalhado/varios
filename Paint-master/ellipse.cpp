/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "shape.h"

namespace Paint {

class Ellipse : public Shape
{
public:

    explicit Ellipse(const QPoint &topLeft,
                     int penWidth,
                     const QColor& penColor);

protected:
    virtual void doDraw(QPainter &painter) override;

    virtual QRect doRect() const override;

    virtual void doUpdate(const QPoint &toPoint) override;

private:
    QRect r;
};

Ellipse::Ellipse(const QPoint &topLeft, int penWidth, const QColor &penColor) :
    Shape(penWidth, penColor), r(topLeft, topLeft)
{
}

void Ellipse::doDraw(QPainter &painter)
{
    if (!r.isNull()) {
        painter.drawEllipse(r.normalized());
    }
}

QRect Ellipse::doRect() const
{
    return r.normalized();
}

void Ellipse::doUpdate(const QPoint &toPoint)
{
    r.setBottomRight(toPoint);
}

std::unique_ptr<Shape> createEllipse(const QPoint &topLeft,
                                     int penWidth,
                                     const QColor& penColor)
{
    return std::unique_ptr<Shape>(new Ellipse(topLeft, penWidth, penColor));
}

} // namespace Paint
