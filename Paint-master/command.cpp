/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "command.h"

#include "document.h"

namespace Paint {

/* Needed for merging */
enum CommandId { ResizeCommandId = 1 };

ShapeCommand::ShapeCommand(QWidget *doc, QImage *image,
                           std::unique_ptr<Shape> &&s) :
    doc(doc), image(image),
    undoImage(image->copy(s->rect())), shape(std::move(s))
{
}

void ShapeCommand::undo()
{
    const QRect rect = shape->rect();

    QPainter painter(image);
    painter.drawImage(rect, undoImage);

    doc->update(rect);
}

void ShapeCommand::redo()
{
    QPainter painter(image);
    shape->draw(painter);

    doc->update(shape->rect());
}

FlipCommand::FlipCommand(QWidget *doc, QImage *image,
                         bool horizontal, bool vertical) :
    doc(doc), image(image), horizontal(horizontal), vertical(vertical)
{
}

void FlipCommand::undo()
{
    redo();
}

void FlipCommand::redo()
{
    *image = image->mirrored(horizontal, vertical);
    doc->update();
}

ResizeCommand::ResizeCommand(QWidget *doc, QImage *image, const QSize &size) :
    doc(doc), image(image), oldSize(image->size()), newSize(size)
{
}

int ResizeCommand::id() const
{
    return ResizeCommandId;
}

bool ResizeCommand::mergeWith(const QUndoCommand *command)
{
    if (command->id() != id())
        return false;

    /* Subsequent resize command will look like a single one:  keep the first
     * command's previous size and update it with the latest resize command's
     * new size.
     */
    newSize = static_cast<const ResizeCommand*>(command)->newSize;
    return true;
}

void ResizeCommand::undo()
{
    *image = image->copy(0, 0, oldSize.width(), oldSize.height());
    doc->update();
}

void ResizeCommand::redo()
{
    if (image->size() == newSize) {
        return;
    }

    QImage newImage(newSize, QImage::Format_RGB32);
    newImage.fill(Qt::white);

    QPainter painter(&newImage);
    painter.drawImage(QPoint(0, 0), *image);
    *image = newImage;

    doc->update();
}

RotateCommand::RotateCommand(QWidget *doc, QImage *image, qreal deg) :
    doc(doc), image(image), deg(deg)
{
}

void RotateCommand::undo()
{
    rotate(-deg);
}

void RotateCommand::redo()
{
    rotate(deg);
}

void RotateCommand::rotate(qreal deg)
{
    QMatrix transf;
    *image = image->transformed(transf.rotate(deg));

    doc->update();
}

} // namespace Paint
