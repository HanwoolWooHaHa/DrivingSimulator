#include "../../Include/Navigation/normalPFM.h"
#include "../../Include/database.h"
#include "../../Include/lineExtractor.h"

#include <qmath.h>
#include <qdebug.h>

CNormalPFM::CNormalPFM()
{}

CNormalPFM::~CNormalPFM()
{}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
void CNormalPFM::Initialize(void)
{}

int CNormalPFM::CalculateAccelerate(int nTick, bool bLaneChangingFlag, double* pdDataArray, double* pdAccX, double* pdAccY)
{
    double dDelta = 0.0001;

	double dTgtPosX = pdDataArray[14];
	double dTgtPosY = pdDataArray[15];

	double dGoalX = pdDataArray[12];
	double dGoalY = pdDataArray[13];

	
	// 1. Calculate a potential energy from side lines
	double dLinePotential = calcLinePotential(nTick, bLaneChangingFlag, dTgtPosX, dTgtPosY);
	double dLinePotentialDashX = calcLinePotential(nTick, bLaneChangingFlag, dTgtPosX + dDelta, dTgtPosY);

	if (dLinePotential < 0.0 || dLinePotentialDashX < 0.0)
		return -2;


	// 2. Calculate a potential energy by adjacent vehicles
	double dCarPotential = calcCarPotential(nTick, bLaneChangingFlag, dTgtPosX, dTgtPosY, pdDataArray);
	double dCarPotentialDashX = calcCarPotential(nTick, bLaneChangingFlag, dTgtPosX + dDelta, dTgtPosY, pdDataArray);
	double dCarPotentialDashY = calcCarPotential(nTick, bLaneChangingFlag, dTgtPosX, dTgtPosY + dDelta, pdDataArray);

	if (dCarPotential == FAIL || dCarPotentialDashX == FAIL || dCarPotentialDashY == FAIL)
		return FAIL;


    // 3. Calculate a potential energy by the goal
	double dGoalPotential = calcGoalPotential(bLaneChangingFlag, dTgtPosX, dTgtPosY, dGoalX, dGoalY);
	double dGoalPotentialDashX = calcGoalPotential(bLaneChangingFlag, dTgtPosX + dDelta, dTgtPosY, dGoalX, dGoalY);
	double dGoalPotentialDashY = calcGoalPotential(bLaneChangingFlag, dTgtPosX, dTgtPosY + dDelta, dGoalX, dGoalY);


    *pdAccX = -(dLinePotentialDashX - dLinePotential) - (dGoalPotentialDashX - dGoalPotential) - (dCarPotentialDashX - dCarPotential);
	*pdAccX /= dDelta;
	
    *pdAccY = -(dGoalPotentialDashY - dGoalPotential) - (dCarPotentialDashY - dCarPotential);
    *pdAccY /= dDelta;

	return DONE;
}
///////////////////////////////////////////////////////////////////////////
/* Private functions */
///////////////////////////////////////////////////////////////////////////
double CNormalPFM::calcLinePotential(int nTick, bool bLaneChangingFlag, double dTgtPosX, double dTgtPosY)
{
	int nCurrentLane = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_LANE);
	int nLeftLine = nCurrentLane - 2;
	int nRightLine = nCurrentLane - 1;

	if (nCurrentLane <= 0 || nCurrentLane >= 7) // Range of the current lane is 1 to 6
	{
		return FAIL;
	}

	int nSide = 0;
	double dDistanceFromLeftLine = 0.0;
	double dDistanceFromRightLine = 0.0;
	if (nLeftLine < 0)
	{
		dDistanceFromLeftLine = dTgtPosX;
	}
	else
	{
		dDistanceFromLeftLine = CLineExtractor::GetInstance()->DistanceWrtLine(dTgtPosX * METER_TO_FEET, dTgtPosY * METER_TO_FEET, nLeftLine, &nSide) * FEET_TO_METER;
	}
	dDistanceFromRightLine = CLineExtractor::GetInstance()->DistanceWrtLine(dTgtPosX * METER_TO_FEET, dTgtPosY * METER_TO_FEET, nRightLine, &nSide) * FEET_TO_METER;
	

	double dPotential = 0.0;

    //double dOmega = 5.0;
	//double dSigma = 1.6;

	double dOmega = 3.0;
	double dSigma = 0.3 * LANE_WIDTH * FEET_TO_METER;

	if (bLaneChangingFlag)
	{
		int nTargetLane = CDatabase::GetInstance()->GetDataInfo(TARGET, DATA_INFO_PACKET_END_LANE);

		if (nTargetLane > nCurrentLane)
		{
			dDistanceFromRightLine += LANE_WIDTH * FEET_TO_METER;
		}
		else if (nTargetLane < nCurrentLane)
		{
			dDistanceFromLeftLine += LANE_WIDTH * FEET_TO_METER;
		}
	}

	double dPotentialLeft = dOmega * qExp(-(dDistanceFromLeftLine*dDistanceFromLeftLine) / (dSigma * dSigma));
	double dPotentialRight = dOmega * qExp(-(dDistanceFromRightLine*dDistanceFromRightLine) / (dSigma * dSigma));

	dPotential = dPotentialLeft + dPotentialRight;

	return dPotential;
}

double CNormalPFM::calcCarPotential(int nTick, bool bLaneChangingFlag, double dPosX, double dPosY, double* pdDataArray)
{
	double dPotential = 0.0;
	int nNumAdjVehicles = CDatabase::GetInstance()->GetNumAdjacentVehicles();

	if (bLaneChangingFlag) // Consider four adjacent vehicles
	{
		double dPotentialPreceding = 0.0;
		double dPotentialFollowing = 0.0;
		double dPotentialLead = 0.0;
		double dPotentialRear = 0.0;

        // Coefficient for the repulsive potential energy by adjacent vehicles
        double dSigmaX = 5.0;
        double dSigmaY = 17.4;
		double dOmega = 12.2;

        double dTgtLength = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_LENGTH) * FEET_TO_METER;
        double dTgtWidth = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_WIDTH) * FEET_TO_METER;

		for (int n = 0; n < nNumAdjVehicles; n++)
		{
			int nVehicleNo = CDatabase::GetInstance()->GetDataInfo(ADJACENT, n, 0);

			if (nVehicleNo == pdDataArray[PRECEDING_VEHICLE_NO])
			{
				double dDelta = pdDataArray[15] - dPosY;
				double dGap = pdDataArray[PRECEDING_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
					dGap = 0.0;

				dPotentialPreceding = dOmega * qExp(-(dGap * dGap) / (dSigmaY * dSigmaY));
			}
			else if (nVehicleNo == pdDataArray[FOLLOWING_VEHICLE_NO])
			{
				double dDelta = dPosY - pdDataArray[15];
				double dGap = pdDataArray[FOLLOWING_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
					dGap = 0.0;

				dPotentialFollowing = dOmega * qExp(-(dGap * dGap) / (dSigmaY * dSigmaY));
			}
			else if (nVehicleNo == pdDataArray[LEAD_VEHICLE_NO])
			{
				double dDelta = pdDataArray[15] - dPosY;
				double dGap = pdDataArray[LEAD_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
                {
                    //qDebug() << "Collision with lead vehicle";
					return FAIL;
                }

				dPotentialLead = dOmega * qExp(-(dGap * dGap) / (dSigmaY * dSigmaY));
#if 0
                double dAdjPosX = CDatabase::GetInstance()->GetData(ADJACENT, n, nTick, ADJ_DATA_PACKET_X) * FEET_TO_METER;
                double dAdjWidth = CDatabase::GetInstance()->GetDataInfo(ADJACENT, n, 3) * FEET_TO_METER;

                double dGapX = qAbs(dAdjPosX - dPosX) - 0.5 * dAdjWidth - 0.5 * dTgtWidth;
				double dGapY = pdDataArray[LEAD_VEHICLE_GAP];
                
                dPotentialLead = dOmega * qExp(-(dGapX * dGapX) / (dSigmaX * dSigmaX)-(dGapY * dGapY) / (dSigmaY * dSigmaY));
#endif
			}
			else if (nVehicleNo == pdDataArray[REAR_VEHICLE_NO])
			{
				double dDelta = dPosY - pdDataArray[15];
				double dGap = pdDataArray[REAR_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
                {
                    //qDebug() << "Collision with rear vehicle";
					return FAIL;
                }

				dPotentialRear = dOmega * qExp(-(dGap * dGap) / (dSigmaY * dSigmaY));
			}
			else
			{
				continue;
			}
		}

		dPotential = dPotentialPreceding + dPotentialFollowing + dPotentialLead + dPotentialRear;
	}
	else // Only consider vehicles that is driving on the current lane
	{
		double dPotentialPreceding = 0.0;
		double dPotentialFollowing = 0.0;

        // Coefficient for the repulsive potential energy by adjacent vehicles
        double dSigma = 17.4;
		double dOmegaP = 12.2;
        double dOmegaF = 0.5 * dOmegaP;

        double dTgtLength = CDatabase::GetInstance()->GetData(TARGET, 0, nTick, DATA_PACKET_LENGTH) * FEET_TO_METER;

        for (int n = 0; n < nNumAdjVehicles; n++)
        {
            int nVehicleNo = CDatabase::GetInstance()->GetDataInfo(ADJACENT, n, 0);

            if (nVehicleNo == pdDataArray[PRECEDING_VEHICLE_NO])
            {
				double dDelta = pdDataArray[15] - dPosY;
				double dGap = pdDataArray[PRECEDING_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
					dGap = 0.0;

				dPotentialPreceding = dOmegaP * qExp(-(dGap * dGap) / (dSigma * dSigma));
            }
            else if (nVehicleNo == pdDataArray[FOLLOWING_VEHICLE_NO])
            {
				double dDelta = dPosY - pdDataArray[15];
				double dGap = pdDataArray[FOLLOWING_VEHICLE_GAP] + dDelta;

				if (dGap < 0.0)
					dGap = 0.0;
				
                dPotentialFollowing = dOmegaF * qExp(-(dGap * dGap) / (dSigma * dSigma));
            }
            else
            {
                continue;
            }
        }

		dPotential = dPotentialPreceding + dPotentialFollowing;
	}

	return dPotential;
}

double CNormalPFM::calcGoalPotential(bool bLaneChangingFlag, double dPosX, double dPosY, double dGoalX, double dGoalY)
{
    double dPotential = 0.0;

    double dWgX = 0.5;
    double dWgY = 1.0;

    double dPotentialX = 0.0;
    double dPotentialY = dWgY * qAbs(dGoalY - dPosY);
	
    if(bLaneChangingFlag)
        dPotentialX = dWgX * qAbs(dGoalX - dPosX);

    dPotential = dPotentialX + dPotentialY;

    return dPotential;
}
