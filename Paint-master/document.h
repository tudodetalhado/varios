/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#ifndef DOCUMENT_H
#define DOCUMENT_H

#include "shape.h"

#include <QUndoStack>
#include <QWidget>

#include <functional>
#include <memory>

namespace Paint {

/**
 * @brief The Document class representing a document where we draw on.
 */
class Document : public QWidget
{
    Q_OBJECT
public:

    /**
     * @brief Shapes factory function prototype
     */
    typedef std::function<
        std::unique_ptr<Shape>(
            const QPoint&, int, const QColor&)> shape_factory_t;

    explicit Document(QUndoStack *undoStack, QWidget *parent = 0);

    bool isModified() const;

    bool openImage(const QString &fileName);
    bool saveImage(const QString &fileName, const char *fileFormat);

    void setPenColor(const QColor &newColor);
    void setPenWidth(int newWidth);

    QColor getPenColor() const { return penColor; }
    int getPenWidth() const { return penWidth; }

    void setShapeFactory(shape_factory_t f);

    void flip(bool horiz, bool vert);
    void rotate(qreal deg);

    std::vector<QPoint> floodFill(const QPoint &pos, const QRgb &color);

protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);

private:
    QImage image;
    QUndoStack *undoStack;

    int penWidth;
    QColor penColor;

    shape_factory_t factory;

    std::unique_ptr<Shape> currentShape;
};

} // namespace Paint

#endif // DOCUMENT_H
