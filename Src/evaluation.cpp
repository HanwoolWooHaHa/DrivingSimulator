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

    m_dAvgErrorSum = m_dAvgErrorSumX = m_dAvgErrorSumY = 0.0;
    m_nAvgCounter = 0;
}

CEvaluation::~CEvaluation()
{

}

void CEvaluation::CalcTrajectoryPredictionError(int nTick)
{
	int nGroundTruth = CDatabase::GetInstance()->GetGroundTruth();
	int nDataLength = CDatabase::GetInstance()->GetDataLength();

	// Check ground truth
	if (nGroundTruth < 25)
	{
		qDebug() << "evaluation.cpp @ Ground truth is less than 25.";
		return;
	}

	if ((nGroundTruth + 25) >= nDataLength)
	{
		qDebug() << "evaluation.cpp @ Ground truth is greater than data length.";
		return;
	}

	int nEvaluationStartingTime = nGroundTruth - 25;
	int nEvaluationFinishingTime = nGroundTruth + 25;

	// Check current time
	if (nTick < nEvaluationStartingTime || nTick > nEvaluationFinishingTime)
		return;

	double dErrorSum = 0.0;
    double dErrorSumX = 0.0;
    double dErrorSumY = 0.0;

	for (int i = 0; i < (int)(TRAJECTORY_PREDICTION_TIME * 10); i++)
	{
		double dPosX_GroundTruth = CDatabase::GetInstance()->GetData(TARGET, 0, nTick + i, DATA_PACKET_X) * FEET_TO_METER;
		double dPosY_GroundTruth = CDatabase::GetInstance()->GetData(TARGET, 0, nTick + i, DATA_PACKET_Y) * FEET_TO_METER;

		double dPosX_Predicted = 0.0;
		double dPosY_Predicted = 0.0;

		CDatabase::GetInstance()->GetPredictedTrajectory(i, &dPosX_Predicted, &dPosY_Predicted);

		double dErrorX = dPosX_Predicted - dPosX_GroundTruth;
		double dErrorY = dPosY_Predicted - dPosY_GroundTruth;
		double dError = qSqrt(dErrorX*dErrorX + dErrorY*dErrorY);

		int nDeltaTime = nTick - nEvaluationStartingTime;
		m_dRecordData[m_nNumRecordedVehicles][nDeltaTime][i][0] = dPosX_GroundTruth;
		m_dRecordData[m_nNumRecordedVehicles][nDeltaTime][i][1] = dPosY_GroundTruth;
		m_dRecordData[m_nNumRecordedVehicles][nDeltaTime][i][2] = dPosX_Predicted;
		m_dRecordData[m_nNumRecordedVehicles][nDeltaTime][i][3] = dPosY_Predicted;
		m_dRecordData[m_nNumRecordedVehicles][nDeltaTime][i][4] = dError;
		
		dErrorSum += dError;
        dErrorSumX += dErrorX;
        dErrorSumY += dErrorY;
	}

	double dAvgError = dErrorSum / (TRAJECTORY_PREDICTION_TIME * 10);
    double dAvgErrorX = dErrorSumX / (TRAJECTORY_PREDICTION_TIME * 10);
    double dAvgErrorY = dErrorSumY / (TRAJECTORY_PREDICTION_TIME * 10);

    m_dAvgErrorSum += dAvgError;
    m_dAvgErrorSumX += dAvgErrorX;
    m_dAvgErrorSumY += dAvgErrorY;

    m_nAvgCounter++;

	qDebug() << "evaluation.cpp @ t = " << nTick << " : Avg.Error = " << dAvgError;

	if (nTick == nEvaluationFinishingTime)
	{
		saveEvaluationResult();
		m_nNumRecordedVehicles++;
	}
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

void CEvaluation::initialize(void)
{
	m_nNumRecordedVehicles = 0;

	memset(m_dRecordData, 0, sizeof(double) * NUM_TRAFFIC_DATA * 60 * 50 * 5);
}

void CEvaluation::saveEvaluationResult(void)
{
	int nVehicleNo = CDatabase::GetInstance()->GetData(TARGET, 0, 0, DATA_PACKET_NO);

    QString file = "/Users/woohanwool/Program/Log/Prediction/" + QString::number(nVehicleNo) + ".csv";

	QFile* fp = new QFile(file);
	if (!fp->open(QIODevice::WriteOnly))
	{
		delete fp;
		return;
	}

	QTextStream* out = new QTextStream(fp);

	for (int t = 0; t < 51; t++)
	{
		for (int i = 0; i < (TRAJECTORY_PREDICTION_TIME * 10); i++)
		{
			for (int k = 0; k < 5; k++)
			{
				*out << m_dRecordData[m_nNumRecordedVehicles][t][i][k] << ",";
			}
			*out << endl;
		}
	}

	fp->close();

	delete fp;
	delete out;
}
