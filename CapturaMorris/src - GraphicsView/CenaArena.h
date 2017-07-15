#ifndef CENAARENA_H
#define CENAARENA_H

#include <QGraphicsScene>
#include <ItemArena.h>

class ArenaCena : public QGraphicsScene
{

public:
    enum Mode {
        NoMode, SelectObject, DrawLine
    };

    ArenaCena(QObject* parent = 0);
    void setMode(Mode mode);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);


private:
    Mode sceneMode;
    QPointF origPoint;
    ItemArena* itemToDraw;
    void makeItemsControllable(bool areControllable);

};
#endif // CENAARENA_H
