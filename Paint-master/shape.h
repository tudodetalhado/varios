/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#ifndef SHAPE_H
#define SHAPE_H

#include <QColor>
#include <QPainter>
#include <QPoint>
#include <QRect>

#include <memory>

namespace Paint {

class Document;

/**
 * @brief The Shape base class representing all shapes drawn on the document.
 */
class Shape
{
public:
    Shape(int penWidth, const QColor& penColor);

    virtual ~Shape();

    /**
     * @brief Draw the shape using a painter.
     * @param painter The painter to use for drawing.
     */
    void draw(QPainter &painter);

    /**
     * @brief Get the rectangle containing the shape.
     * @return The shape's rectangle.
     */
    QRect rect() const;

    /**
     * @brief Update the shape while "stretching" (drawing) it
     * @param toPoint The last recorded point where the shape should be extended
     *                to.
     */
    void update(const QPoint &toPoint);

protected:
    virtual void doDraw(QPainter &painter) = 0;

    virtual QRect doRect() const = 0;

    virtual void doUpdate(const QPoint &toPoint) = 0;

private:
    int penWidth;
    QColor penColor;
};

/**
 * To keep it terse in this toy project, all shape classes are defined in
 * '*.cpp' files.  This also aids better code locality.  In a bigger design
 * better modularity would be preferred.  Here only factory functions are
 * exposed.
 */

std::unique_ptr<Shape> createEllipse(const QPoint &topLeft,
                                     int penWidth,
                                     const QColor& penColor);

std::unique_ptr<Shape> createEraser(const QPoint &topLeft,
                                      int penWidth,
                                      const QColor&);

std::unique_ptr<Shape> createFill(Document *doc,
                                  const QPoint &topLeft,
                                  int penWidth,
                                  const QColor&);

std::unique_ptr<Shape> createRectangle(const QPoint &topLeft,
                                       int penWidth,
                                       const QColor& penColor);

std::unique_ptr<Shape> createScribble(const QPoint &topLeft,
                                      int penWidth,
                                      const QColor& penColor);

} // namespace Paint

#endif // SHAPE_H
