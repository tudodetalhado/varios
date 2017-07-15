/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#ifndef COMMAND_H
#define COMMAND_H

#include "shape.h"

#include <QImage>
#include <QWidget>
#include <QUndoCommand>

#include <memory>
#include <vector>

namespace Paint {

/**
 * Here we have all commands to be used for undo/redo support.
 */

/**
 * @brief The ShapeCommand class for creating/undoing shapes on the document.
 */
class ShapeCommand : public QUndoCommand
{
public:
    explicit ShapeCommand(QWidget *doc, QImage *image,
                          std::unique_ptr<Shape> &&s);

    virtual void undo() override;
    virtual void redo() override;

private:
    QWidget *doc;
    QImage *image;

    QImage undoImage;
    std::unique_ptr<Shape> shape;
};

/**
 * @brief The FlipCommand class for flipping the image (document).  Can flip
 * both horizontally and vertically in any combination.
 */
class FlipCommand : public QUndoCommand
{
public:
    explicit FlipCommand(QWidget *doc, QImage *image,
                         bool horizontal, bool vertical);

    virtual void undo() override;
    virtual void redo() override;

private:
    QWidget *doc;
    QImage *image;

    bool horizontal, vertical;
};

/**
 * @brief The ResizeCommand class for resizing the image (document).  Only
 * increases the size.  Able to be merged with other subsequent resize commands.
 */
class ResizeCommand : public QUndoCommand
{
public:
    explicit ResizeCommand(QWidget *doc, QImage *image, const QSize &size);

    virtual int id() const override;
    virtual bool mergeWith(const QUndoCommand *command) override;

    virtual void undo() override;
    virtual void redo() override;

private:
    QWidget *doc;
    QImage *image;

    QSize oldSize, newSize;
};

/**
 * @brief The RotateCommand class for rotating the image (document).
 */
class RotateCommand : public QUndoCommand
{
public:
    explicit RotateCommand(QWidget *doc, QImage *image, qreal deg);

    virtual void undo() override;
    virtual void redo() override;

private:
    void rotate(qreal deg);

    QWidget *doc;
    QImage *image;

    qreal deg;
};

} // namespace Paint

#endif // COMMAND_H
