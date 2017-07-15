/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <QImage>

#include <vector>

namespace Paint {

/**
 * @brief Find all points on the image that need to be changed in order to get
 * them flood-filled.
 *
 * @param img An image to apply flood-fill to.  Doesn't get modified.
 * @param pos A point to start the flood from.
 * @param newColor A new color to apply.
 * @return List of points that would be changed in the original image.
 */
std::vector<QPoint>
floodFill(QImage *image, const QPoint &pos, const QRgb &newColor);

} // namespace Paint

#endif // FLOODFILL_H
