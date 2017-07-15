/* $Id: fill.c,v 1.3 2004/12/25 04:41:58 meffie Exp $
 *
 * GNU Paint
 * Copyright 2000-2003, 2007  Li-Cheng (Andy) Tai
 *
 * Authors: Li-Cheng (Andy) Tai
 *          Michael A. Meffie III <meffiem@neo.rr.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Paint
 * Copyright (C) 2014 Krzysztof Konopko <krzysztof.konopko@konagma.pl>
 */

#include "floodfill.h"

#include <vector>

namespace Paint {

/**
 * The whole flood-fill thing is extracted from gPaint's 'src/fill.c' source.
 * Left as close as possible to the original code with some required
 * modifications (eg. using `std::vector` as the stack rather than a
 * preallocated buffer).
 */

struct fillpixelinfo
{
   int y, xl, xr, dy;
};

#define PUSH(py, pxl, pxr, pdy) \
{ \
    struct fillpixelinfo p;\
    if (((py) + (pdy) >= 0) && ((py) + (pdy) < image.height()))\
    {\
        p.y = (py);\
        p.xl = (pxl);\
        p.xr = (pxr);\
        p.dy = (pdy);\
        stack.push_back(p); \
    }\
}

#define POP(py, pxl, pxr, pdy) \
{\
    struct fillpixelinfo p = stack.back();\
    stack.pop_back();\
    (py) = p.y + p.dy;\
    (pxl) = p.xl;\
    (pxr) = p.xr;\
    (pdy) = p.dy;\
}

std::vector<QPoint>
floodFill(QImage *img, const QPoint &pos, const QRgb &newColor)
{
   QImage image = img->copy();
   std::vector<QPoint> modified;

   int x = pos.x(), y = pos.y();
   const QRgb oldColor = image.pixel(x, y);

   std::vector<fillpixelinfo> stack;

   int l, x1, x2, dy;

   if ((x >= 0) && (x < image.width()) && (y >= 0) && (y < image.height()))
   {
       if (oldColor == newColor)
       {
           return modified;
       }
       PUSH(y, x, x, 1);
       PUSH(y + 1, x, x, -1);
       while (!stack.empty())
       {
           POP(y, x1, x2, dy);
           for (x = x1; (x >= 0) && image.pixel(x, y) == oldColor; x--)
           {
               image.setPixel(x, y, newColor);
               modified.emplace_back(x, y);
           }
           if (x >= x1)
           {
               goto skip;
           }
           l = x + 1;
           if (l < x1)
           {
               PUSH(y, l, x1 - 1, -dy);
           }
           x = x1 + 1;
           do
           {
               for (; (x < image.width()) && image.pixel(x, y) == oldColor; x++)
               {
                   image.setPixel(x, y, newColor);
                   modified.emplace_back(x, y);
               }
               PUSH(y, l, x - 1, dy);
               if (x > x2 + 1)
               {
                   PUSH(y, x2 + 1, x - 1, -dy);
               }
skip:
               for (x++; x <= x2 && image.pixel(x, y) != oldColor; x++)
               {
                   /* empty */ ;
               }
               l = x;
           } while (x <= x2);
       }
   }

   return modified;
}

} // namespace Paint
