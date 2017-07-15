/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "document.h"
#include "shape.h"

#include <QPoint>

#include <vector>

namespace Paint {

class Fill : public Shape
{
public:

    explicit Fill(Document *doc,
                  const QPoint &topLeft,
                  int penWidth,
                  const QColor& penColor);

protected:
    virtual void doDraw(QPainter &painter) override;

    virtual QRect doRect() const override;

    virtual void doUpdate(const QPoint &toPoint) override;

private:
    std::vector<QPoint> points;
    QRect r;
};

Fill::Fill(Document *doc,
           const QPoint &topLeft,
           int penWidth,
           const QColor &penColor) :
    Shape(penWidth, penColor),
    points(doc->floodFill(topLeft, penColor.rgb())),
    r(topLeft, topLeft)
{
    // Find the containing rectangle
    for (const QPoint &p : points) {
        if (p.x() < r.left()) {
            r.setLeft(p.x());
        } else if (p.x() > r.right()) {
            r.setRight(p.x());
        }

        if (p.y() < r.top()) {
            r.setTop(p.y());
        } else if (p.y() > r.bottom()) {
            r.setBottom(p.y());
        }
    }
}

void Fill::doDraw(QPainter &painter)
{
    painter.drawPoints(points.data(), points.size());
}

QRect Fill::doRect() const
{
    return r;
}

void Fill::doUpdate(const QPoint &)
{
    // Flood-fill at the point of button press.  Ignore "dragging".
}

std::unique_ptr<Shape> createFill(Document *doc,
                                  const QPoint &topLeft,
                                  int penWidth,
                                  const QColor& penColor)
{
    return std::unique_ptr<Shape>(new Fill(doc, topLeft, penWidth, penColor));
}

} // namespace Paint
