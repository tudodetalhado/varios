Paint
=====

This is a toy project to try out some Qt5 features.  The aim was to create a
simple GUI application that resembles MS Paint or gPaint tools.  Particularly
the following features were kept in mind:

* drawing lines, rectangles and ellipses

* changing pen width and color

* changing the window size and supporting images of different size

* support reading/writing BMP format

* support at least 3 undo/redo operations

* flood fill operation

* flipping vertically and horizontally

* rotating left and right by 90 degrees, rotating by 180 degrees

* erase operation


Design
======

A lot of problems are solved by the Qt library itself.  Some patterns also feel
natural when using Qt (e.g. the undo stack and commands).  Nevertheless, here is
some design outline.

* MainWindow

  Main application window.  Holds a single Document where the actual drawing
  activity takes place.  Responsible for providing usual GUI app gubbins like
  menus, all sort of dialogues (picking pen color and width, choosing file to
  open/save) etc.

* Document

  Document receives mouse and paint events and provides the canvas for drawing.

  What's important about it is that it uses QImage as a sort of graphics
  "repository":  when the user is done with creating a shape, it's "committed"
  onto the QImage.  Then any widget repaints need only copy the required
  rectangle area from the QImage and optionally draw the shape "in progress".

  The Document owns a Shape "in progress", i.e. the one that the user is
  "dragging".  When it's done with it (the user releases left mouse button), it
  moves the Shape to a ShapeCommand which is pushed onto the undo stack.  Every
  Shape is started (created) by a factory method provided by the MainWidow based
  on the brush shape option (menu).

  The Document also turns other operations into undo commands and pushes them
  onto the undo stack.  Usually the commands take the Document itself and its
  QImage pointers to be able to manipulate them directly.

* Shape

  Shape objects can draw themselves onto the Document's QImage.  This is used
  by undo/redo commands.  Each Shape also provides its containing rectangle to
  find out which part of the Document's canvas needs to be updated when the
  shape is removed.

* Undo/Redo commands

  Ideally each command should be able to undo/redo its result.  Qt's Undo
  framework takes care of the rest.

  The challenge is to find a way to remove a Shape from the image.  The way this
  is achieved is to save the bit of the image covered by the Shape's rectangle
  before drawing the actual Shape.

  Undoing the Shape means drawing "back" saved image rectangle while redoing it
  simply means drawing the Shape (again).

  All other operations are reversible by their nature (e.g. undoing Flip simply
  means doing it again).

* Flood fill

  This touches upon some clever [graphics algorithms][1].  To avoid getting it
  wrong, a seemingly efficient implementation has been borrowed from [gPaint
  project][2].

  To support undo/redo, flood-fill algorithm collects all pixels which should be
  modified and returns them to the caller.  Then they can be used to do actually
  "draw" them on the image when needed.

  [1]: http://en.wikipedia.org/wiki/Flood_fill/
  [2]: http://www.gnu.org/software/gpaint/

Building and testing
====================
This project has been built and run successfully on Debian/Wheezy with Qt-5.2.0.
The binary has also been run successfully on Mint/Olivia with the following
libraries taken from the Qt installation:
* libicudata.so.51
* libicui18n.so.51
* libicuuc.so.51
* libQt5Core.so.5
* libQt5DBus.so.5
* libQt5Gui.so.5
* libQt5Widgets.so.5
* platforms/libqxcb.so

Given that the files above are located in the current directory along with the
executable itself, this whole business can be run as follows:

    LD_LIBRARY_PATH=. ./Paint
