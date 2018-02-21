#include "../../Include/Navigation/sinusoidal.h"
#include "../../Include/defines.h"
#include "../../Include/database.h"

#include <qmath.h>

CSinusoidalModel::CSinusoidalModel()
{}

CSinusoidalModel::~CSinusoidalModel()
{}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
void CSinusoidalModel::Initialize()
{}

int CSinusoidalModel::CalculateAccelerate(int nTick, bool bLaneChangingFlag, double* pdDataArray, double* pdAccX, double* pdAccY)
{
    double dDelta = 0.0001;

    double dTgtPosX = pdDataArray[14];
    double dTgtPosY = pdDataArray[15];

    double dGoalX = pdDataArray[12];
    double dGoalY = pdDataArray[13];


    // 1. Calculate the lateral acceleration by the sinusoidal model
    *pdAccX = calcSinusoidal(bLaneChangingFlag, nTick);

    // 2. Calculate a potential energy by adjacent vehicles
    double dCarPotential = calcCarPotential(bLaneChangingFlag, dTgtPosY, pdDataArray);
    double dCarPotentialDashY = calcCarPotential(bLaneChangingFlag, dTgtPosY + dDelta, pdDataArray);

    if (dCarPotential == FAIL || dCarPotentialDashY == FAIL)
        return FAIL;


    // 3. Calculate a potential energy by the goal
    double dGoalPotential = calcGoalPotential(bLaneChangingFlag, dTgtPosX, dTgtPosY, dGoalX, dGoalY);
    double dGoalPotentialDashY = calcGoalPotential(bLaneChangingFlag, dTgtPosX, dTgtPosY + dDelta, dGoalX, dGoalY);


    // 4. Update the acceleration in both X and Y direction
    *pdAccY = -(dGoalPotentialDashY - dGoalPotential) - (dCarPotentialDashY - dCarPotential);
    *pdAccY /= dDelta;

    *pdAccX = 0.0;
    *pdAccY = 0.0;

    return DONE;
}
///////////////////////////////////////////////////////////////////////////
/* Private member functions */
///////////////////////////////////////////////////////////////////////////
double CSinusoidalModel::calcSinusoidal(bool bLaneChangingFlag, int nTick)
{
    static bool bLaneChangingInitialFlag = false;
    static int nStartTick = 0;

    double dAccX = 0.0;

    // If the vehicle does not change a lane, the acceleration in X direction is zero
    if(bLaneChangingFlag == false)
    {
        bLaneChangingFlag = false;
        nStartTick = 0;

        return dAccX;
    }

    // Save the moment when a lane change is started
    double dLaneChangingDuration = 46; // average is 4.6 s
    if(bLaneChangingInitialFlag == false)
    {
        nStartTick = nTick;
        bLaneChangingInitialFlag = true;
    }

    // Calculate the lateral acceleration
    dAccX = (2 * M_PI * LANE_WIDTH * FEET_TO_METER) / (dLaneChangingDuration * dLaneChangingDuration) * qSin(2 * M_PI * (double)(nTick - nStartTick) / dLaneChangingDuration);

    return dAccX;
}

double CSinusoidalModel::calcCarPotential(bool bLaneChangingFlag, double dPosY, double* pdDataArray)
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
        double dSigmaY = 17.4;
        double dOmega = 12.2;

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

double CSinusoidalModel::calcGoalPotential(bool bLaneChangingFlag, double dPosX, double dPosY, double dGoalX, double dGoalY)
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
