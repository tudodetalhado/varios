#include "ArenaView.h"

ArenaView::ArenaView(QWidget *parent) :
    QGraphicsView(parent)
{
    //setRenderHints(QPainter::Antialiasing);
}

void ArenaView::mousePressEvent(QMouseEvent *event)
{
    _exibirCor = _exibirCor == true ? false : true;
}

bool ArenaView::exibirCor()
{
   return _exibirCor;
}

void ArenaView::setCena(ArenaCena *cena)
{
    this->cena = cena;
    setScene(cena);
}



ArenaCena *ArenaView::getCena()
{
   return this->cena;
}

void  ArenaView::leaveEvent(QEvent *event)
{
   emit desativarFerramentas();
}

void ArenaView::mouseDoubleClickEvent(QMouseEvent *e)
{
    emit cliqueDuplo();
}


