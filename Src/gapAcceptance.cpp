#include "../Include/gapAcceptance.h"
#include "../Include/database.h"
#include "../Include/calcNormalCum.h"
//#include "../Include/drivingStyleEstimator.h"

#include <qmath.h>
#include <QDebug>

#define STD_LEAD_GAP 1.112
#define STD_REAR_GAP 0.742

CGapAcceptance::CGapAcceptance()
{
    m_dDriverCoefficient = 0.0;
}

CGapAcceptance::~CGapAcceptance()
{

}
///////////////////////////////////////////////////////////////////////////
//! Public functions
///////////////////////////////////////////////////////////////////////////

double CGapAcceptance::Predict(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dTgtX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
    double dPrecedPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

    double dGap = dPrecedPosX - dTgtX;


    if( dGap >= 50.0 )
    {
        CDatabase::GetInstance()->SetGapAcceptanceProbability( 0.5 );
        return DONE;
    }


    m_dDriverCoefficient = 1.112;

#ifdef DRIVING_STYLE_RECOGNITION
    int nDrivingStyleClass = CDrivingStyleEstimator::GetInstance()->Estimate(nTick);

    if(nDrivingStyleClass == FAIL)
    {
        qDebug() << "Gap Acceptance @ Driving style estimation was failed...";
        qDebug() << nPrecedVehicleNo << ", " << dPrecedGap;
    }

    CDatabase::GetInstance()->SetDrivingStyle(nDrivingStyleClass);

    switch(nDrivingStyleClass)
    {
    case STYLE_CAUTIOUS: m_dDriverCoefficient = 0.5; break;
    case STYLE_AGGRESSIVE: m_dDriverCoefficient = -1.0; break;
    default: m_dDriverCoefficient = 0.0; break;
    }
#endif


    double dCriticalGap = calcCrticalGapLead( nTick );
    double dProbLead = calcProbLead( nTick );

    CDatabase::GetInstance()->SetCriticalLeadGap( dCriticalGap );
    CDatabase::GetInstance()->SetGapAcceptanceProbability( dProbLead );

    return DONE;
}

void CGapAcceptance::Initialize( void )
{
    CDatabase::GetInstance()->SetCriticalLeadGap( 0.0 );
    CDatabase::GetInstance()->SetGapAcceptanceProbability( 0.5 );
}

///////////////////////////////////////////////////////////////////////////
//! Private member functions
///////////////////////////////////////////////////////////////////////////
double CGapAcceptance::calcCrticalGapLead(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dTgtVel = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
    double dLeadVel = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_V);
    double dDeltaV = dLeadVel - dTgtVel;

    double dCriticalGap = qExp( 1.353 - 2.700 * qMax(0.0, dDeltaV) - 0.231 * qMin(0.0, dDeltaV) + 1.270 * m_dDriverCoefficient );

    if( dCriticalGap <= 0.0 )
    {
        qDebug() << "GapAcceptance @ t=" << nTick << "CalcCriticalGapLead has some problem...";
    }

    return dCriticalGap;
}

double CGapAcceptance::calcProbLead( int nTick )
{
    double dProb = FAIL;

    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dTgtX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
    double dLeadPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_X);

    double dGap = dLeadPosX - dTgtX;
    double dCriticalGap = CDatabase::GetInstance()->GetCriticalLeadGap();

    if(dGap < 0.0 || dCriticalGap==0.0)
    {
        return 0.5;
    }

    //! Calculate the probability to change a lane based on the gap acceptrance model
    dProb = CNormalCum::GetInstance()->CumNormal( (qLn(dGap) - qLn(dCriticalGap)) / STD_LEAD_GAP );

    if(dProb > 1.0)
        dProb = 1.0;
    if(dProb < 0.0)
        dProb = 0.0;


    return dProb;
}
