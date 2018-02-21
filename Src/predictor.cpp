/**
* @file	predictor.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/08/04
* @brief
*/

#include "../Include/predictor.h"
#include "../Include/database.h"
#include "../Include/Navigation/normalPFM.h"
#include "../Include/Navigation/sinusoidal.h"

#include <qdebug.h>
#include <qmath.h>

CPredictor::CPredictor(void)
{
    //! Use the potential field method
    m_pNaviMethod = new CNormalPFM();

    //! Use the sinusoidal model
    //m_pNaviMethod = new CSinusoidalModel();
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
	double arrdData[20] = { 0.0 };
	for (int i = 0; i < 20; i++)
	{
		arrdData[i] = CDatabase::GetInstance()->GetAdjacentVehicleData(i);
	}

	double dTgtPosX = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_X) * FEET_TO_METER;
	double dTgtPosY = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_Y) * FEET_TO_METER;
	double dTgtVelX = 0.0;
	double dTgtVelY = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_VEL) * FEET_TO_METER;
	if (nTick != 0)
	{
		double dPreTgtPosX = CDatabase::GetInstance()->GetData(TARGET, 0, nTick - 1, DATA_PACKET_X) * FEET_TO_METER;
		dTgtVelX = (dTgtPosX - dPreTgtPosX) * 10.0;
		//qDebug() << "predictor.cpp @ t = " << nTick << " : VelX = " << dTgtVelX << ",  VelY = " << dTgtVelY << ",  Ang = " << qRadiansToDegrees(qAtan(qAbs(dTgtVelX) / qAbs(dTgtVelY)));
	}

	// 1. Set the goal position referred the result of the lane change estimation
	bool bLaneChangingFlag = CDatabase::GetInstance()->GetLaneChangingFlag();
	double dGoalPosX = 0.0;
	double dGoalPosY = 0.0;
	setGoalPosition(bLaneChangingFlag, dTgtPosX, dTgtPosY, &dGoalPosX, &dGoalPosY);

	arrdData[12] = dGoalPosX;
	arrdData[13] = dGoalPosY;
	arrdData[14] = dTgtPosX;
	arrdData[15] = dTgtPosY;
	arrdData[16] = dTgtVelX;
	arrdData[17] = dTgtVelY;

	//! Check the vehicle gap with adjacent vehicles
	//qDebug() << "predictor.cpp @ t = " << nTick << " : P=" << arrdData[PRECEDING_VEHICLE_GAP] << ", F=" << arrdData[FOLLOWING_VEHICLE_GAP] << ", L=" << arrdData[LEAD_VEHICLE_GAP] << ", R=" << arrdData[REAR_VEHICLE_GAP];

	calculateAverageVelocity(nTick, arrdData);

	double dDelta = 0.1;
	int nUpdateCounter = 0;
	bool bCollision = false;

	// 2. Conduct a trajectory prediction
	for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
	{
		double dTgtAccX = 0.0;
		double dTgtAccY = 0.0;

		// 2.1 Calculate an acceleration of the target vehicle
		int nReturn = m_pNaviMethod->CalculateAccelerate(nTick, bLaneChangingFlag, arrdData, &dTgtAccX, &dTgtAccY);
		if (nReturn == FAIL)
		{
			//qDebug() << "predictor.cpp @ Lane changing is dangerous!!!";

			bCollision = true;
			//break;
		}
		
		// 2.2 Update positions of vehicles
		update(bLaneChangingFlag, arrdData, dTgtAccX, dTgtAccY, dDelta);

		// 2.3 Save the predicted trajectory
		CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdData[14], arrdData[15]); // save the position of the target vehicle : (PosX, PosY [m])

		nUpdateCounter++;
	}

#if 1
	nUpdateCounter = 0;
	if (bCollision)
	{
		int nIndex = 0;
		// if collision occures, conduct the trajectory prediction again setting goal position on the current lane
		for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
		{
			double dPosX = 0.0;
			double dPosY = 0.0;

			CDatabase::GetInstance()->GetPredictedTrajectory(nIndex, &dPosX, &dPosY);

			CDatabase::GetInstance()->SetRePredictedTrajectory(nIndex, dPosX, dPosY); // save the position of the target vehicle : (PosX, PosY [m])

			nIndex++;
		}

		for (int i = 0; i < 20; i++)
		{
			arrdData[i] = CDatabase::GetInstance()->GetAdjacentVehicleData(i);
		}

		setGoalPosition(false, dTgtPosX, dTgtPosY, &dGoalPosX, &dGoalPosY);

		arrdData[12] = dGoalPosX;
		arrdData[13] = dGoalPosY;
		arrdData[14] = dTgtPosX;
		arrdData[15] = dTgtPosY;
		arrdData[16] = dTgtVelX;
		arrdData[17] = dTgtVelY;

		for (double t = 0.0; t < TRAJECTORY_PREDICTION_TIME; t += dDelta)
		{
			double dTgtAccX = 0.0;
			double dTgtAccY = 0.0;

			// 2.1 Calculate an acceleration of the target vehicle
			m_pNaviMethod->CalculateAccelerate(nTick, false, arrdData, &dTgtAccX, &dTgtAccY);

			// 2.2 Update positions of vehicles
			update(false, arrdData, dTgtAccX, dTgtAccY, dDelta);

			// 2.3 Save the predicted trajectory
			CDatabase::GetInstance()->SetPredictedTrajectory(nUpdateCounter, arrdData[14], arrdData[15]); // save the position of the target vehicle : (PosX, PosY [m])

			nUpdateCounter++;
		}
	}
#endif

	return bCollision;
}
///////////////////////////////////////////////////////////////////////////
/* Private functions */
///////////////////////////////////////////////////////////////////////////
void CPredictor::setGoalPosition(bool bLaneChangingFlag, double dTgtPosX, double dTgtPosY, double* pdGoalPosX, double* pdGoalPosY)
{
	int nStartLane = (int)CDatabase::GetInstance()->GetDataInfo(TARGET, DATA_INFO_PACKET_START_LANE);
	int nEndLane = (int)CDatabase::GetInstance()->GetDataInfo(TARGET, DATA_INFO_PACKET_END_LANE);

	*pdGoalPosY = dTgtPosY + 20.0;
	
	if (bLaneChangingFlag)
	{
		*pdGoalPosX = ((nEndLane - 1) * LANE_WIDTH + 0.5 * LANE_WIDTH) * FEET_TO_METER;
	}
	else
	{
		*pdGoalPosX = ((nStartLane - 1) * LANE_WIDTH + 0.5 * LANE_WIDTH) * FEET_TO_METER;
	}
}

void CPredictor::update(bool bLaneChangingFlag, double* arrdData, double dTgtAccX, double dTgtAccY, double dDelta)
{
	double dTgtPosX = arrdData[14];
	double dTgtPosY = arrdData[15];
	double dTgtVelX = arrdData[16];
	double dTgtVelY = arrdData[17];

	
	int nLeadVehicleNo = (int)arrdData[LEAD_VEHICLE_NO];
	if (nLeadVehicleNo != 0)
	{
		double dLeadGap = arrdData[LEAD_VEHICLE_GAP];
		double dLeadRelVelY = arrdData[LEAD_VEHICLE_REL_VEL];

		dLeadGap += dLeadRelVelY * dDelta;

		arrdData[LEAD_VEHICLE_GAP] = dLeadGap;
	}

	int nPrecedingVehicleNo = (int)arrdData[PRECEDING_VEHICLE_NO];
	if (nPrecedingVehicleNo != 0)
	{
		double dPrecedingGap = arrdData[PRECEDING_VEHICLE_GAP];
		double dPrecedingRelVelY = arrdData[PRECEDING_VEHICLE_REL_VEL];

		dPrecedingGap += dPrecedingRelVelY * dDelta;

		arrdData[PRECEDING_VEHICLE_GAP] = dPrecedingGap;
	}

	int nRearVehicleNo = (int)arrdData[REAR_VEHICLE_NO];
	if (nRearVehicleNo != 0)
	{
		double dRearGap = arrdData[REAR_VEHICLE_GAP];
		double dRearRelVelY = arrdData[REAR_VEHICLE_REL_VEL];

		dRearGap += dRearRelVelY * dDelta;

		arrdData[REAR_VEHICLE_GAP] = dRearGap;
	}

	int nFollowingVehicleNo = (int)arrdData[FOLLOWING_VEHICLE_NO];
	if (nFollowingVehicleNo != 0)
	{
		double dFollowingGap = arrdData[FOLLOWING_VEHICLE_GAP];
		double dFollowingRelVelY = arrdData[FOLLOWING_VEHICLE_REL_VEL];

		dFollowingGap += dFollowingRelVelY * dDelta;

		arrdData[FOLLOWING_VEHICLE_GAP] = dFollowingGap;
	}

	//! Check the limit of velocity X
	double dAngle = qRadiansToDegrees(qAbs(dTgtVelX) / qAbs(dTgtVelY));
	if (dAngle > 4.0 && bLaneChangingFlag == false) // Limit Vel X is only adapted during the lane keeping.
	{
		dTgtVelX = qAbs(dTgtVelY) * qTan(qDegreesToRadians(4.0)) * (dTgtVelX / qAbs(dTgtVelX));
	}

	//! Check the velocity Y
	if (dTgtVelY < 0.0)
		dTgtVelY = 0.0;

	// Update
	dTgtPosX += dTgtVelX * dDelta;
	dTgtPosY += dTgtVelY * dDelta;
	dTgtVelX += dTgtAccX * dDelta;
	dTgtVelY += dTgtAccY * dDelta;


	// Save
	arrdData[14] = dTgtPosX;
	arrdData[15] = dTgtPosY;
	arrdData[16] = dTgtVelX;
	arrdData[17] = dTgtVelY;
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
