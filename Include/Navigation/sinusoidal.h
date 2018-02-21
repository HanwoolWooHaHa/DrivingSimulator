#ifndef SINUSOIDAL_H
#define SINUSOIDAL_H

#include "navigationMethod.h"

class CSinusoidalModel : public CNavigationMethod
{
public:
    CSinusoidalModel();
    ~CSinusoidalModel();

    virtual void Initialize(void);
    virtual int CalculateAccelerate(int nIntention, double* parrdData, double* pdAccX, double* pdAccY);

private:

    double calcSinusoidal(int nIntention, double* parrdData);
};

#endif // SINUSOIDAL_H
