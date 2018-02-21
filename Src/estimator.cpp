/**
* @file	estimator.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/04/26
* @brief	this class is implemented for the estimation algorithm using sensor information
*/

#include "../Include/estimator.h"
#include "../Include/database.h"
#include "../Include/method.h"
#include "../Include/mySVM.h"
//#include "../Include/myHmm.h"

#include <qmath.h>
#include <qdebug.h>
/*********************************************************************/
CEstimator::CEstimator(int nMethod)
{
    m_pDatabase = CDatabase::GetInstance();

	switch (nMethod)
	{
	case SVM:
        m_pMethod = new CMySvm();
		break;

	case HMM:
        //m_pMethod = new CMyHmm();
		break;

    case TRAJECTORY:
        m_pMethod = NULL;
        break;

    default:
        m_pMethod = NULL;
        break;
	}
}

CEstimator::~CEstimator()
{
	delete m_pDatabase;

    if( m_pMethod != NULL)
        delete m_pMethod;
}
/*********************************************************************/
/* Public functions */
void CEstimator::Estimate( int nTick, int nMode )
{
    int nEstimationResult = DEFAULT;

    if(m_pMethod == NULL)
    {
        nEstimationResult = test(nTick);
        CDatabase::GetInstance()->SetDrivingIntention( nEstimationResult );
    }
    else
        m_pMethod->Test( nTick, nMode );
}
/*********************************************************************/
/* Private member functions */
int CEstimator::test( int nTick )
{
    static int nPreIntention = DEFAULT;
    int nIntention = DEFAULT;
    int nIndexPredictedTrajectory = (int)(TRAJECTORY_PREDICTION_TIME / DS_TRJ_PRD_DELTA); // prediction time X 10 Hz;

    if(nTick == 0)
        nPreIntention = DEFAULT;

    double dCurrentPosX = 0.0;
    double dCurrentPosY = 0.0;
    double dPosX = 0.0;
    double dPosY = 0.0;

    CDatabase::GetInstance()->GetPredictedTrajectory(0, &dCurrentPosX, &dCurrentPosY);
    CDatabase::GetInstance()->GetPredictedTrajectory(nIndexPredictedTrajectory, &dPosX, &dPosY);


    if(dCurrentPosY > DS_CENTERLINE && dPosY > DS_CENTERLINE)
    {
        if(nPreIntention==RETURNING || nPreIntention==ADJUSTMENT)
            nIntention = ADJUSTMENT;
        else
            nIntention = KEEPING;
    }
    else if(dCurrentPosY > DS_CENTERLINE && dPosY <= DS_CENTERLINE)
        nIntention = CHANGING;
    else if(dCurrentPosY <= DS_CENTERLINE && dPosY <= DS_CENTERLINE)
        nIntention = ARRIVAL;
    else if(dCurrentPosY <= DS_CENTERLINE && dPosY > DS_CENTERLINE)
        nIntention = RETURNING;
    else
        nIntention = DEFAULT;

    nPreIntention = nIntention;

    return nIntention;
}
