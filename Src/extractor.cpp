/**
* @file	extractor.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/07/23
* @brief	this file works to extract features using traffic data or measurement data
*/

#include "../Include/extractor.h"
#include "../Include/database.h"
#include "../Library/libpf/potential.h"
#include "../Include/calcNormalCum.h"
//#include "../Include/defines.h"

#include <qmath.h>

CExtractor::CExtractor()
{

}

CExtractor::~CExtractor()
{

}
/*********************************************************************/
/* Public functions */
void CExtractor::Initialize()
{
    m_nSide = 1;
}

void CExtractor::Extract( int nTick )
{
    double dFeatureValue[FEATURE_VECTOR_DIMENSION] = { 0.0 };
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    //! Extract the first feature : distance to the centerline
    dFeatureValue[FEATURE_PACKET_DISTANCE] = calcDistanceToCenterline( nTick );

    //! Extract the second feature : lateral velocity
    if( nTick == 0 )
    {
        dFeatureValue[FEATURE_PACKET_LAT_VELOCITY] = 0.0;
    }
    else
    {
        dFeatureValue[FEATURE_PACKET_LAT_VELOCITY] = calcLateralVelocity( nTick );
    }

    //! Extract the third feature : potential feature
    if( FEATURE_VECTOR_DIMENSION == 3 )
    {
        double dPotentialValue = calculatePotentialValue( nTick );

        CDatabase::GetInstance()->SetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_POTENTIAL, dPotentialValue);
        dFeatureValue[FEATURE_PACKET_POTENTIAL] = dPotentialValue;
    }

    //! Conduct scaling using the standard deviation
    dFeatureValue[FEATURE_PACKET_DISTANCE] /= DS_FEATURE_LAT_DST_STD;
    dFeatureValue[FEATURE_PACKET_LAT_VELOCITY] /= DS_FEATURE_LAT_VEL_STD;

    CDatabase::GetInstance()->SetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_DISTANCE, dFeatureValue[FEATURE_PACKET_DISTANCE]);
    CDatabase::GetInstance()->SetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_LAT_VELOCITY, dFeatureValue[FEATURE_PACKET_LAT_VELOCITY]);
}

void CExtractor::ChangeSide(int nSide)
{
    m_nSide = nSide;
}

/*********************************************************************/
/* Private member functions */
double CExtractor::calcDistanceToCenterline(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    //! Extract the first feature : distance to the centerline
    double dPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_Y); //! lateral position

    double dDistance = dPosY - DS_CENTERLINE;

    dDistance *= m_nSide;

    return dDistance;
}

double CExtractor::calcLateralVelocity(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dDistance = calcDistanceToCenterline(nTick);
    double dPreDistance = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick-1, FEATURE_PACKET_DISTANCE) * DS_FEATURE_LAT_DST_STD;

    double dLateralVel = (dDistance - dPreDistance) / DS_DELTA_T;

    return dLateralVel;
}

double CExtractor::calculatePotentialValue( int nTick )
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X); //! longitudinal position
    double dPrecedingX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);
    double dLeadX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_X);

    double dVelX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V); //! longitudinal velocity
    double dPrecedingVel = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_V);
    double dLeadVel = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_V);

    double dPrecedingGap = dPrecedingX - dPosX;
    double dLeadGap = dLeadX - dPosX;

    double dPrecedingRelVel = dPrecedingVel - dVelX;
    double dLeadRelVel = dLeadVel - dVelX;


    double dPotentialP = CPotential::GetInstance()->Field( dPrecedingGap, dPrecedingRelVel, 180.0 );
    double dPotentialL = CPotential::GetInstance()->Field( dLeadGap, dLeadRelVel, 180.0 );

    double dValue = 0.0;

    if( dPotentialL == 0.0 && dPotentialP != 0.0 )
    {
        dValue = 1.0;
    }
    else if( dPotentialL == 0.0 && dPotentialP == 0.0 )
    {
        dValue = 0.5;
    }
    else
    {
        double dRatio = dPotentialP / dPotentialL;

        if(dRatio >= 100.0)
            dValue = 1.0;
        else if(dRatio == 0.0)
            dValue = 0.0;
        else
            dValue = CNormalCum::GetInstance()->CumNormal( qLn( dRatio ) );
    }

    return dValue;
}
