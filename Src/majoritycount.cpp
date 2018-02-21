#include "../Include/majoritycount.h"
#include "../Include/database.h"

#include "qmath.h"
#include <QDebug>

CMajorityCount::CMajorityCount()
{
    for(int n=0; n<NUM_CLASS; n++)
    {
        m_nSuccess[n] = m_nFail[n] = 0;
    }
}

CMajorityCount::~CMajorityCount()
{

}

int CMajorityCount::Count(int nTrial, int nTick)
{
    int nResult = 0;

    if(nTick < MAJORITY_COUNT_SIZE)
    {
        nResult = CDatabase::GetInstance()->GetDsClassificationResult(nTrial, nTick);
    }
    else
    {
        int nCounter[NUM_CLASS] = { 0 };

        for(int t=0; t<MAJORITY_COUNT_SIZE; t++)
        {
            int nClass = CDatabase::GetInstance()->GetDsClassificationResult(nTrial, nTick-t);

            nCounter[nClass-1]++;
        }

        int nMaxValue = 0;
        int nMaxClass = 0;
        for(int n=0; n<NUM_CLASS; n++)
        {
            int nValue = nCounter[n];
            if(nValue > nMaxValue)
            {
                nMaxValue = nCounter[n];
                nMaxClass = n;
            }
        }

        nResult = nMaxClass + 1;
    }

    int nGroundTruth = CDatabase::GetInstance()->GetGroundTruth();

    if(nGroundTruth == nResult)
        m_nSuccess[nGroundTruth-1]++;
    else
        m_nFail[nGroundTruth-1]++;

    return nResult;
}

void CMajorityCount::PrintAccuracy(void)
{
    for(int n=0; n<NUM_CLASS; n++)
    {
        m_dAccuracy[n] = (double)(m_nSuccess[n]) / (double)(m_nSuccess[n] + m_nFail[n]);
        qDebug() << "Majority @ Accuracy " << n << " : " << m_dAccuracy[n];
    }
}
