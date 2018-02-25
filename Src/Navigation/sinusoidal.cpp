#include "../../Include/Navigation/sinusoidal.h"
#include "../../Include/defines.h"
#include "../../Include/database.h"

#include <qmath.h>
#include <qdebug.h>

CSinusoidalModel::CSinusoidalModel()
{}

CSinusoidalModel::~CSinusoidalModel()
{}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
void CSinusoidalModel::Initialize()
{}

int CSinusoidalModel::CalculateAccelerate(int nIntention, double* parrdData, double* pdAccX, double* pdAccY, int nUpdateCounter)
{
    //! Calculate the lateral acceleration by the sinusoidal model
    *pdAccY = calcSinusoidal(nIntention, parrdData);
    *pdAccX = 0.0;

    return DONE;
}
///////////////////////////////////////////////////////////////////////////
/* Private member functions */
///////////////////////////////////////////////////////////////////////////
double CSinusoidalModel::calcSinusoidal(int nIntention, double* parrdData)
{
    double dPosY = parrdData[1];
    double dVelY = parrdData[3];
    double dGoalY = 0.0;

    double dAccY = 0.0;

    // If the vehicle does not change a lane, the acceleration in X direction is zero
    if(nIntention == KEEPING || nIntention == ADJUSTMENT)
    {
        return dAccY;
    }
    else if(nIntention == CHANGING || nIntention == ARRIVAL )
    {
        dGoalY = DS_CENTERLINE - 0.5 * DS_LANE_WIDTH;
    }

    // Calculate the lateral acceleration
    double dLaneChangingDuration = qAbs(DS_LANE_WIDTH / dVelY);
    double dCurrentT = dLaneChangingDuration - qAbs((dGoalY - dPosY) / dVelY);

    dAccY = (2 * M_PI * DS_LANE_WIDTH) / (dLaneChangingDuration * dLaneChangingDuration) * qSin(2 * M_PI * dCurrentT / dLaneChangingDuration);
    dAccY *= -1;

    return dAccY;
}
