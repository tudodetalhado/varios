#ifndef CRONOMETRO_H
#define CRONOMETRO_H

#include <QElapsedTimer>
#include <QTime>

class Cronometro {
    QElapsedTimer timer;

  public:
    Cronometro()
    {
    }

    void reiniciar()
    {
        timer.restart();
    }

    QTime getTime() {
        return QTime(0,0).addMSecs(timer.elapsed());
    }
};

#endif // CRONOMETRO_H
