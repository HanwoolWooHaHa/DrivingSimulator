#include "../Include/graph.h"

#include <QPainter>

CGraph::CGraph(QWidget *parent)
{
    m_nElapsed = 0;
    setFixedSize(500, 300);
    m_bCallRender = false;
    m_nVehicleNo = 0;
}

void CGraph::Update(int nTick)
{
    m_Helper.Update(nTick);
    update();
}

void CGraph::SaveImage(int nVehicleNo)
{
    m_bCallRender = true;
    m_nVehicleNo = nVehicleNo;
}

void CGraph::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    m_Helper.paint(&painter, event);
    painter.end();

    if(m_bCallRender)
    {
        QPixmap pixmap(this->size());
        this->render(&pixmap);
        pixmap.save("../Log/Image/no"+QString::number(m_nVehicleNo)+".png");
        m_bCallRender = false;
    }
}
