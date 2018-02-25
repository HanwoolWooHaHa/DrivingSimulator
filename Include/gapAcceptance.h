#ifndef GAPACCEPTANCE_H
#define GAPACCEPTANCE_H

#include "../Include/defines.h"

#define STYLE_CAUTIOUS 1
#define STYLE_AGGRESSIVE 2

class CGapAcceptance
{
public:
    static CGapAcceptance* GetInstance()
    {
        static CGapAcceptance* instance = new CGapAcceptance();
        return instance;
    }

    ~CGapAcceptance();

    double Predict(int nTick);
    void Initialize( void );

private:
    CGapAcceptance();

    double calcCrticalGapLead( int nTick );

    double calcProbLead( int nTick );

    double m_dDriverCoefficient;
};

#endif // GAPACCEPTANCE_H
