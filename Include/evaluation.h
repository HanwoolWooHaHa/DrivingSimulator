#pragma once

#include "defines.h"

/**
* @class	CEvaluation
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2018/02/20 \n
* 			    Last revision date: 2018/02/20 HanwoolWoo
* @brief	this class is for saving measured data
*/

class CEvaluation
{
public:
	static CEvaluation* GetInstance()
	{
		static CEvaluation* instance = new CEvaluation();
		return instance;
	}

	~CEvaluation();

	void CalcTrajectoryPredictionError(int nTick);
    void CalcTrajectoryPredictionError3s(int nTick);
    double GetAvgError(int nIndex); // 0: AVG in Both of X, Y direction, 1: X direction, 2: Y direction
    int GetNumRecordedVehicles(void);
    double GetData(int nVehicleIndex, int nTime, int nPredictionTime, int nColumn);
    void InitializeAvgError(void);
    void PrintAvgError(void);

private:
	CEvaluation();

    void initialize(void);

	int m_nNumRecordedVehicles;
	double m_dRecordData[NUM_TRAFFIC_DATA][60][50][5];
    double m_dEvaluationPoints[100][4];

    double m_dAvgErrorSum;
    double m_dAvgErrorSumX;
    double m_dAvgErrorSumY;
    int m_nAvgCounter;
};
