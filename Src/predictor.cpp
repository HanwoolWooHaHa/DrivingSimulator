/**
* @file	predictor.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/08/04
* @brief
*/

#include "../Include/predictor.h"
#include "../Include/database.h"
//#include "../Include/Navigation/normalPFM.h"
#include "../Include/Navigation/sinusoidal.h"

#include <qdebug.h>
#include <qmath.h>

CPredictor::CPredictor(void)
{
    m_pNaviMethod = NULL;

    //! Use the potential field method
    //m_pNaviMethod = new CNormalPFM();

    //! Use the sinusoidal model
    m_pNaviMethod = new CSinusoidalModel();
}

CPredictor::~CPredictor(void)
{
	delete m_pNaviMethod;
}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
bool CPredictor::Predict(int nTick)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();
    int nNumTrial = CDatabase::GetInstance()->GetNumTrial();

    if (nCurrentTrial < 0 || nCurrentTrial > nNumTrial)
        nCurrentTrial = 0;

    double dTgtPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X); // direction of travel, longitudinal position
    double dTgtPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_Y); // lateral position
	double dTgtVelX = 0.0;
    double dTgtVelY = 0.0;

	if (nTick != 0)
	{
        double dPreTgtPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_OWN_X);
        double dPreTgtPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_OWN_Y);

        dTgtVelX = (dTgtPosX - dPreTgtPosX) * 120.0; // 120 Hz = measurement period
        dTgtVelY = (dTgtPosY - dPreTgtPosY) * 120.0; // 120 Hz = measurement period

        //qDebug() << "predictor.cpp @ t = " << nTick << " : VelX = " << dTgtVelX << ",  VelY = " << dTgtVelY;
	}

    double arrdTgtInfo[4] = { dTgtPosX, dTgtPosY, dTgtVelX, dTgtVelY };


    double dDelta = DS_TRJ_PRD_DELTA;
    bool bLaneChangingFlag = false;
    int nUpdateCounter = 0;

    // save the current position
    CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdTgtInfo[0], arrdTgtInfo[1]);
    nUpdateCounter++;

    // Conduct a trajectory prediction
	for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
	{
        double dTgtAccX = 0.0;
        double dTgtAccY = 0.0;

        // Update positions of vehicles
        update( arrdTgtInfo, dTgtAccX, dTgtAccY, dDelta );

        // Save the predicted trajectory
        CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdTgtInfo[0], arrdTgtInfo[1]);

        nUpdateCounter++;
	}


    bool bCollision = false;
	return bCollision;
}

bool CPredictor::Predict( int nTick, int nIntention )
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();
    int nNumTrial = CDatabase::GetInstance()->GetNumTrial();

    if (nCurrentTrial < 0 || nCurrentTrial > nNumTrial)
        nCurrentTrial = 0;


    //! Predict the target's trajectory
    double dTgtPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X); // direction of travel, longitudinal position
    double dTgtPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_Y); // lateral position
    double dTgtVelX = 0.0;
    double dTgtVelY = 0.0;

    if (nTick != 0)
    {
        double dPreTgtPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_OWN_X);
        double dPreTgtPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_OWN_Y);

        dTgtVelX = (dTgtPosX - dPreTgtPosX) * 120.0; // 120 Hz = measurement period
        dTgtVelY = (dTgtPosY - dPreTgtPosY) * 120.0; // 120 Hz = measurement period

        //qDebug() << "predictor.cpp @ t = " << nTick << " : VelX = " << dTgtVelX << ",  VelY = " << dTgtVelY;
    }

    double arrdTgtInfo[4] = { dTgtPosX, dTgtPosY, dTgtVelX, dTgtVelY };


    double dDelta = DS_TRJ_PRD_DELTA;
    int nUpdateCounter = 0;

    // save the current position
    CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdTgtInfo[0], arrdTgtInfo[1]);
    nUpdateCounter++;

    // Conduct the trajectory prediction
    for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
    {
        double dTgtAccX = 0.0;
        double dTgtAccY = 0.0;

        m_pNaviMethod->CalculateAccelerate(nIntention, arrdTgtInfo, &dTgtAccX, &dTgtAccY);

        // Update positions of vehicles
        update( arrdTgtInfo, dTgtAccX, dTgtAccY, dDelta );


//        if(nIntention == ARRIVAL && (arrdTgtInfo[1] < DS_CENTERLINE - 0.5 * DS_LANE_WIDTH))
//            arrdTgtInfo[1] = DS_CENTERLINE - 0.5 * DS_LANE_WIDTH;
//        if(nIntention == ADJUSTMENT && (arrdTgtInfo[1] > DS_CENTERLINE + 0.5 * DS_LANE_WIDTH))
//            arrdTgtInfo[1] = DS_CENTERLINE + 0.5 * DS_LANE_WIDTH;


        // Save the predicted trajectory
        CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdTgtInfo[0], arrdTgtInfo[1]);

        nUpdateCounter++;
    }



    //! Predict the preceding vehicle's trajectory
    double dPrcdPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X); // direction of travel, longitudinal position
    double dPrcdPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_Y); // lateral position
    double dPrcdVelX = 0.0;
    double dPrcdVelY = 0.0;

    if (nTick != 0)
    {
        double dPrePrcdPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_PRECED_X);
        double dPrePrcdPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_PRECED_Y);

        dPrcdVelX = (dPrcdPosX - dPrePrcdPosX) * 120.0; // 120 Hz = measurement period
        dPrcdVelY = (dPrcdPosY - dPrePrcdPosY) * 120.0; // 120 Hz = measurement period
    }

    double arrdPrcdInfo[4] = { dPrcdPosX, dPrcdPosY, dPrcdVelX, dPrcdVelY };


    nUpdateCounter = 0;

    // save the current position
    CDatabase::GetInstance()->SetPrecedingTrajectory(nUpdateCounter, arrdPrcdInfo[0], arrdPrcdInfo[1]);
    nUpdateCounter++;

    // Conduct the trajectory prediction
    for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
    {
        double dAccX = 0.0;
        double dAccY = 0.0;

        // Update positions of vehicles
        update( arrdPrcdInfo, dAccX, dAccY, dDelta );

        // Save the predicted trajectory
        CDatabase::GetInstance()->SetPrecedingTrajectory(nUpdateCounter, arrdPrcdInfo[0], arrdPrcdInfo[1]);

        nUpdateCounter++;
    }



    //! Predict the lead vehicle's trajectory
    double dLeadPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_X); // direction of travel, longitudinal position
    double dLeadPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_LEAD_Y); // lateral position
    double dLeadVelX = 0.0;
    double dLeadVelY = 0.0;

    if (nTick != 0)
    {
        double dPreLeadPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_LEAD_X);
        double dPreLeadPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick-1, DS_LEAD_Y);

        dLeadVelX = (dLeadPosX - dPreLeadPosX) * 120.0; // 120 Hz = measurement period
        dLeadVelY = (dLeadPosY - dPreLeadPosY) * 120.0; // 120 Hz = measurement period
    }

    double arrdLeadInfo[4] = { dLeadPosX, dLeadPosY, dLeadVelX, dLeadVelY };


    nUpdateCounter = 0;

    // save the current position
    CDatabase::GetInstance()->SetLeadTrajectory(nUpdateCounter, arrdLeadInfo[0], arrdLeadInfo[1]);
    nUpdateCounter++;

    // Conduct the trajectory prediction
    for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
    {
        double dAccX = 0.0;
        double dAccY = 0.0;

        // Update positions of vehicles
        update( arrdLeadInfo, dAccX, dAccY, dDelta );

        // Save the predicted trajectory
        CDatabase::GetInstance()->SetLeadTrajectory(nUpdateCounter, arrdLeadInfo[0], arrdLeadInfo[1]);

        nUpdateCounter++;
    }


    bool bCollision = false;
    return bCollision;
}

///////////////////////////////////////////////////////////////////////////
/* Private functions */
///////////////////////////////////////////////////////////////////////////
void CPredictor::update(double* arrdData, double dTgtAccX, double dTgtAccY, double dDelta)
{
    double dTgtPosX = arrdData[0];
    double dTgtPosY = arrdData[1];
    double dTgtVelX = arrdData[2];
    double dTgtVelY = arrdData[3];

    /*
    //! Check the limit of velocity X
	double dAngle = qRadiansToDegrees(qAbs(dTgtVelX) / qAbs(dTgtVelY));
	if (dAngle > 4.0 && bLaneChangingFlag == false) // Limit Vel X is only adapted during the lane keeping.
	{
		dTgtVelX = qAbs(dTgtVelY) * qTan(qDegreesToRadians(4.0)) * (dTgtVelX / qAbs(dTgtVelX));
	}

	//! Check the velocity Y
	if (dTgtVelY < 0.0)
		dTgtVelY = 0.0;
    */


	// Update
    dTgtVelX += dTgtAccX * dDelta;
    dTgtVelY += dTgtAccY * dDelta;
    dTgtPosX += dTgtVelX * dDelta;
    dTgtPosY += dTgtVelY * dDelta;


	// Save
    arrdData[0] = dTgtPosX;
    arrdData[1] = dTgtPosY;
    arrdData[2] = dTgtVelX;
    arrdData[3] = dTgtVelY;
}

void CPredictor::calculateAverageVelocity(int nTick, double* pdAdjData)
{
	static double dRecordData[10][4] = { 0.0 };

	if (nTick < 9)
	{
		dRecordData[nTick][0] = pdAdjData[LEAD_VEHICLE_REL_VEL];
		dRecordData[nTick][1] = pdAdjData[REAR_VEHICLE_REL_VEL];
		dRecordData[nTick][2] = pdAdjData[PRECEDING_VEHICLE_REL_VEL];
		dRecordData[nTick][3] = pdAdjData[FOLLOWING_VEHICLE_REL_VEL];

		double dSum[4] = { 0.0 };
		for (int i = 0; i <= nTick; i++)
		{
			dSum[0] += dRecordData[i][0];
			dSum[1] += dRecordData[i][1];
			dSum[2] += dRecordData[i][2];
			dSum[3] += dRecordData[i][3];
		}

		pdAdjData[LEAD_VEHICLE_REL_VEL] = dSum[0] / (nTick + 1);
		pdAdjData[REAR_VEHICLE_REL_VEL] = dSum[1] / (nTick + 1);
		pdAdjData[PRECEDING_VEHICLE_REL_VEL] = dSum[2] / (nTick + 1);
		pdAdjData[FOLLOWING_VEHICLE_REL_VEL] = dSum[3] / (nTick + 1);
	}
	else
	{
		for (int i = 0; i < 9; i++)
		{
			dRecordData[i][0] = dRecordData[i + 1][0];
			dRecordData[i][1] = dRecordData[i + 1][1];
			dRecordData[i][2] = dRecordData[i + 1][2];
			dRecordData[i][3] = dRecordData[i + 1][3];
		}

		dRecordData[9][0] = pdAdjData[LEAD_VEHICLE_REL_VEL];
		dRecordData[9][1] = pdAdjData[REAR_VEHICLE_REL_VEL];
		dRecordData[9][2] = pdAdjData[PRECEDING_VEHICLE_REL_VEL];
		dRecordData[9][3] = pdAdjData[FOLLOWING_VEHICLE_REL_VEL];

		double dSum[4] = { 0.0 };
		for (int i = 0; i < 10; i++)
		{
			dSum[0] += dRecordData[i][0];
			dSum[1] += dRecordData[i][1];
			dSum[2] += dRecordData[i][2];
			dSum[3] += dRecordData[i][3];
		}

		pdAdjData[LEAD_VEHICLE_REL_VEL] = dSum[0] * 0.1;
		pdAdjData[REAR_VEHICLE_REL_VEL] = dSum[1] * 0.1;
		pdAdjData[PRECEDING_VEHICLE_REL_VEL] = dSum[2] * 0.1;
		pdAdjData[FOLLOWING_VEHICLE_REL_VEL] = dSum[3] * 0.1;
	}
}
