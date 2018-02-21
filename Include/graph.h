#ifndef GRAPH_H
#define GRAPH_H

#include <QWidget>
#include "../Include/helper.h"

class CGraph : public QWidget
{
    Q_OBJECT

public:
    CGraph(QWidget* parent=0);
    void Update(int nTick);
    void SaveImage(int nVehicleNo);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

private:
    CHelper m_Helper;
    int m_nElapsed;
    bool m_bCallRender;
    int m_nVehicleNo;
};

#endif // GRAPH_H
