/**
* @file	evaluation.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/09/21
* @brief	this file is for the management of data files
*/
#include "../Include/evaluation.h"
#include "../Include/database.h"

#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <qmath.h>

CEvaluation::CEvaluation()
{
    initialize();
}

CEvaluation::~CEvaluation()
{

}

void CEvaluation::CalcTrajectoryPredictionError(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();
    int nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);


    if(nTick < (5.0 * 120.0)) // evaluation is not performed within 5.0 sec from the start
        return;
    if(nTick > (nDataLength - (int)(5.0 * 120.0)))
        return;



	double dErrorSum = 0.0;
    double dErrorSumX = 0.0;
    double dErrorSumY = 0.0;



    int nIndexPredictedTrajectory = (int)(TRAJECTORY_PREDICTION_TIME / DS_TRJ_PRD_DELTA); // prediction time X 10 Hz;
    int nIndex = (int)(DS_TRJ_DRW_DELTA / DS_TRJ_PRD_DELTA);
    int nNumEvaluationPoints = nIndexPredictedTrajectory / nIndex;
    int nIndexEvaluation = 0;


    for (int i = 0; i <= nIndexPredictedTrajectory; i+=nIndex)
    {
        double dPosX = 0.0;
        double dPosY = 0.0;

        CDatabase::GetInstance()->GetPredictedTrajectory(i, &dPosX, &dPosY);

        m_dEvaluationPoints[nIndexEvaluation][0] = dPosX;
        m_dEvaluationPoints[nIndexEvaluation][1] = dPosY;

        nIndexEvaluation++;
    }



    int nNumIndexGroundTruth = (int)(TRAJECTORY_PREDICTION_TIME / DS_DELTA_T); // prediction time X 120 Hz;
    nIndex = (int)(DS_TRJ_DRW_DELTA / DS_DELTA_T);
    nIndexEvaluation = 0;


    for (int i = 0; i <= nNumIndexGroundTruth; i+=nIndex)
    {
        double dPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick+i, DS_OWN_X);
        double dPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick+i, DS_OWN_Y);

        m_dEvaluationPoints[nIndexEvaluation][2] = dPosX;
        m_dEvaluationPoints[nIndexEvaluation][3] = dPosY;

        nIndexEvaluation++;
    }



    for (int i = 0; i < nNumEvaluationPoints; i++)
	{
        double dErrorX = m_dEvaluationPoints[i][0] - m_dEvaluationPoints[i][2]; // Predicted X - Ground truth X
        double dErrorY = m_dEvaluationPoints[i][1] - m_dEvaluationPoints[i][3]; // Predicted Y - Ground truth Y
		double dError = qSqrt(dErrorX*dErrorX + dErrorY*dErrorY);

        dErrorSum += dError;
        dErrorSumX += qAbs(dErrorX);
        dErrorSumY += qAbs(dErrorY);
	}



    double dAvgError = dErrorSum / nNumEvaluationPoints;
    double dAvgErrorX = dErrorSumX / nNumEvaluationPoints;
    double dAvgErrorY = dErrorSumY / nNumEvaluationPoints;

    m_dAvgErrorSum += dAvgError;
    m_dAvgErrorSumX += dAvgErrorX;
    m_dAvgErrorSumY += dAvgErrorY;

    m_nAvgCounter++;

    //qDebug() << "evaluation.cpp @ t = " << nTick << " : Avg.Error X = " << dAvgErrorX << " : Avg.Error Y = " << dAvgErrorY;
}

double CEvaluation::GetAvgError(int nIndex)
{
    double dValue = 0.0;

    switch(nIndex)
    {
    case 0: dValue = m_dAvgErrorSum / (double)m_nAvgCounter; break;
    case 1: dValue = m_dAvgErrorSumX / (double)m_nAvgCounter; break;
    case 2: dValue = m_dAvgErrorSumY / (double)m_nAvgCounter; break;
    }

    return dValue;
}

int CEvaluation::GetNumRecordedVehicles(void)
{
    return m_nNumRecordedVehicles;
}

double CEvaluation::GetData(int nVehicleIndex, int nTime, int nPredictionTime, int nColumn)
{
    return m_dRecordData[nVehicleIndex][nTime][nPredictionTime][nColumn];
}

void CEvaluation::InitializeAvgError(void)
{
    m_dAvgErrorSum = m_dAvgErrorSumX = m_dAvgErrorSumY = 0.0;
    m_nAvgCounter = 0;
}

void CEvaluation::PrintAvgError(void)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    if(m_nAvgCounter==0)
        return;

    qDebug() << "evaluation.cpp @ Tiral = " << nCurrentTrial << " Avg.Error = " << GetAvgError(0) << " : Avg.Error X = " << GetAvgError(1) << " : Avg.Error Y = " << GetAvgError(2);
}

void CEvaluation::initialize(void)
{
    m_dAvgErrorSum = m_dAvgErrorSumX = m_dAvgErrorSumY = 0.0;
    m_nAvgCounter = 0;

	m_nNumRecordedVehicles = 0;

	memset(m_dRecordData, 0, sizeof(double) * NUM_TRAFFIC_DATA * 60 * 50 * 5);
    memset(m_dEvaluationPoints, sizeof(double), 100 * 4);
}
