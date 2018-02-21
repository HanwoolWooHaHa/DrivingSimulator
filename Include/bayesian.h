#ifndef BAYESIAN_H
#define BAYESIAN_H

#include "defines.h"

class CBayesian
{
public:
    static CBayesian* GetInstance()
    {
        static CBayesian* instance = new CBayesian();
        return instance;
    }

    ~CBayesian() {}

    void Initialize(void);
    int Filter(double* pdProb, double* pdPreProb);

private:
    CBayesian();

    double m_dProbStateTransition[NUM_STATE][NUM_STATE];
};

#endif // BAYESIAN_H
