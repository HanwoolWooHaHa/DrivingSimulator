#ifndef LEASTSQUARE_H
#define LEASTSQUARE_H

#include "../Include/defines.h"
#include <QQueue>

class CLeastSquare
{
public:
    static CLeastSquare* GetInstance()
    {
        static CLeastSquare* instance = new CLeastSquare();
        return instance;
    }

    ~CLeastSquare() {}

    int Estimate(int nTick);
    void Initialize(void);

private:
    CLeastSquare();
    bool isNaN(double x);

    double m_dCv;
    double m_dCs;

    QQueue<double> inputArray0;
    QQueue<double> inputArray1;
    QQueue<double> inputArray2;
    QQueue<double> outputArray;
};

#endif // LEASTSQUARE_H
