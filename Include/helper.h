#ifndef HELPER_H
#define HELPER_H

#include <QBrush>
#include <QFont>
#include <QPen>
#include <QWidget>

class CHelper
{
public:
    CHelper();
    void Update(int nTick) {m_nTick = nTick;}
    void paint(QPainter* painter, QPaintEvent* event);

private:
    int m_nTick;

    QBrush background;
    QPen textPen;
    QPen graphPen;
    QFont textFont;
};

#endif // HELPER_H
