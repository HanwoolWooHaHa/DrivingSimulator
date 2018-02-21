/**
* @file	estimator.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/04/26
* @brief	this class is implemented for the estimation algorithm using sensor information
*/

#include "../Include/estimator.h"
#include "../Include/database.h"
#include "../Include/mySVM.h"
#include "../Include/myHmm.h"

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
        m_pMethod = new CMyHmm();
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
    bool bEstimationResult = m_pMethod->Test( nTick, nMode );

    CDatabase::GetInstance()->SetLaneChangingFlag( bEstimationResult );

    //qDebug() << "estimator.cpp @ t=" << nTick << ",  flag = " << bEstimationResult;
}
/*********************************************************************/
/* Private member functions */
