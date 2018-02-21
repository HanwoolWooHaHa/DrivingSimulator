#include "../Include/bayesian.h"
#include <QDebug>

CBayesian::CBayesian()
{}

void CBayesian::Initialize()
{
    memset(m_dProbStateTransition, 0, sizeof(double) * NUM_STATE * NUM_STATE);

    m_dProbStateTransition[0][0] = 0.629047;
    m_dProbStateTransition[0][1] = 0.349534;
    m_dProbStateTransition[0][2] = 0.021419;

    m_dProbStateTransition[1][0] = 0.356684;
    m_dProbStateTransition[1][1] = 0.536665;
    m_dProbStateTransition[1][2] = 0.106651;

    m_dProbStateTransition[2][0] = 0.025498;
    m_dProbStateTransition[2][1] = 0.195187;
    m_dProbStateTransition[2][2] = 0.779315;
}

int CBayesian::Filter(double* pdProb, double* pdPreProb)
{
    int nState = -1;

    double dProb[NUM_STATE] = {0.0};

    for(int i=0; i<NUM_STATE; i++)
    {
        double dSumProb = 0.0;

        for(int j=0; j<NUM_STATE; j++)
        {
            dSumProb += m_dProbStateTransition[j][i] * pdPreProb[j];
        }

        dProb[i] = pdProb[i] * dSumProb;
    }

    double dSum = 0.0;
    for(int i=0; i<NUM_STATE; i++)
    {
        dSum += dProb[i];
    }

    for(int i=0; i<NUM_STATE; i++)
    {
        dProb[i] = dProb[i] / dSum;
    }

    //! Find the max value and the state
    double dMax = 0.0;
    for(int i=0; i<NUM_STATE; i++)
    {
        if(dProb[i] > dMax)
        {
            dMax = dProb[i];
            nState = i;
        }
    }

    if(nState == -1)
    {
        qDebug() << "bayesian.cpp @ Probability has some problem.";
        return nState;
    }

    //! Copy the pre-probability
    for(int i=0; i<NUM_STATE; i++)
    {
        pdPreProb[i] = pdProb[i];
    }

    for(int i=0; i<NUM_STATE; i++)
    {
        pdProb[i] = dProb[i];
    }

    return nState+1;
}
