#ifndef MAJORITYCOUNT_H
#define MAJORITYCOUNT_H

#include "../Include/defines.h"

class CMajorityCount
{
public:
    static CMajorityCount* GetInstance()
    {
        static CMajorityCount* instance = new CMajorityCount();
        return instance;
    }

    ~CMajorityCount();

    int Count(int nTrial, int nTick);
    void PrintAccuracy( void );

private:
    CMajorityCount();

    int m_nSuccess[NUM_CLASS];
    int m_nFail[NUM_CLASS];

    double m_dAccuracy[NUM_CLASS];
};

#endif // MAJORITYCOUNT_H
