#pragma once
#ifndef ARENAVIEW_H
#define ARENAVIEW_H

#include <CenaArena.h>
#include <QGraphicsView>

class ArenaView : public QGraphicsView
{
    Q_OBJECT
public:
    ArenaView(QWidget *parent);
    ArenaCena* getCena();
    void setCena(ArenaCena *cena);
    bool exibirCor();

protected:
    void leaveEvent(QEvent * event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    ArenaCena *cena;
    bool _exibirCor = false;

signals:
   void desativarFerramentas();
   void cliqueDuplo();

};

#endif // ARENAVIEW_H
