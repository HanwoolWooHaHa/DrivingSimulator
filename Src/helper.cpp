#include "../Include/helper.h"
#include "../Include/database.h"

#include <QPainter>
#include <QPaintEvent>

CHelper::CHelper()
{
    m_nTick = 0;

    background = QBrush(Qt::white);
    textPen = QPen(Qt::black);
    textFont.setPixelSize(22);
    textFont.setBold( true );
    graphPen = QPen(Qt::blue);
    graphPen.setWidth(3.0);
}

void CHelper::paint(QPainter *painter, QPaintEvent *event)
{
    int nTick = m_nTick;

    painter->fillRect(event->rect(), background);

    textPen = QPen(Qt::black);
    textPen.setWidth(2);
    painter->setPen(textPen);
    painter->setFont(textFont);

    painter->drawText(QRect(0, 10, 500, 40), Qt::AlignCenter, "Driving intention");
    painter->drawText(QRect(0, 170, 500, 200), Qt::AlignCenter, "Time");
    //painter->drawText(QRect(0, 200, 500, 230), Qt::AlignCenter, "Time");

    textPen = QPen(Qt::gray);
    textPen.setWidth(1);
    painter->setPen(textPen);

    painter->drawLine(50.0, 50.0, 50.0, 250.0);
    painter->drawLine(50.0, 250.0, 450.0, 250.0);

    painter->drawLine(50.0, 200.0, 450.0, 200.0);
    painter->drawLine(50.0, 150.0, 450.0, 150.0);
    painter->drawLine(50.0, 100.0, 450.0, 100.0);


    painter->setPen(graphPen);

    for(int t=1; t<nTick; t++)
    {
        int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();
        int nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);

        int nT1 = (int)((double)((t-1) * 400.0 / nDataLength));
        int nT2 = (int)((double)(t * 400.0 / nDataLength));

        double dValue1 = CDatabase::GetInstance()->GetEstimatedResult(t-1);
        double dValue2 = CDatabase::GetInstance()->GetEstimatedResult(t);

        int nPosX1 = 50 + nT1;
        int nPosY1 = 300 - (int)(dValue1 * 50.0);
        int nPosX2 = 50 + nT2;
        int nPosY2 = 300 - (int)(dValue2 * 50.0);

        painter->drawLine(nPosX1, nPosY1, nPosX2, nPosY2);
    }
}
