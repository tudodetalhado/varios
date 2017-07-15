#ifndef WINDOW_H
#define WINDOW_H

#include "helper.h"

#include <QWidget>

class Window : public QWidget
{
    Q_OBJECT

public:
    Window();

private:
    Helper helper;
};

#endif
