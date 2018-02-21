#ifndef SINUSOIDAL_H
#define SINUSOIDAL_H

#include "navigationMethod.h"

class CSinusoidalModel : public CNavigationMethod
{
public:
    CSinusoidalModel();
    ~CSinusoidalModel();

    virtual void Initialize(void);
    virtual int CalculateAccelerate(int nTick, bool bLaneChangingFlag, double* pdDataArray, double* pdAccX, double* pdAccY);

private:

    double calcSinusoidal(bool bLaneChangingFlag, int nTick);
    double calcCarPotential(bool bLaneChangingFlag, double dPosY, double* pdDataArray);
    double calcGoalPotential(bool bLaneChangingFlag, double dPosX, double dPosY, double dGoalX, double dGoalY);
};

#endif // SINUSOIDAL_H
