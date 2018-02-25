#include "../../Include/Navigation/normalPFM.h"
#include "../../Include/database.h"

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

int CNormalPFM::CalculateAccelerate(int nIntention, double* parrdData, double* pdAccX, double* pdAccY, int nUpdateCounter)
{
    double dDelta = 0.0001;
    double dPosX = parrdData[0];
    double dPosY = parrdData[1];

    //! Calculate the repulsive potential energy from side lines
    double dLinePotential = calcLinePotential( nIntention, dPosY );
    double dLinePotentialDashY = calcLinePotential( nIntention, dPosY+dDelta );

    //! Calculate the repulsive potential energy from adjacent vehicles
    double dCarPotential = calcCarPotential( nIntention, dPosX, nUpdateCounter );
    double dCarPotentialDashX = calcCarPotential( nIntention, dPosX+dDelta, nUpdateCounter );

    //! Calculate the attractive potential energy from goal
    double dGoalPotential = calcGoalPotential( nIntention, dPosX, dPosY );
    double dGoalPotentialDashX = calcGoalPotential( nIntention, dPosX+dDelta, dPosY );
    double dGoalPotentialDashY = calcGoalPotential( nIntention, dPosX, dPosY+dDelta );



    *pdAccX = -(dGoalPotentialDashX - dGoalPotential) - (dCarPotentialDashX - dCarPotential);
    *pdAccX /= dDelta;

    *pdAccY = -(dLinePotentialDashY - dLinePotential) - (dGoalPotentialDashY - dGoalPotential);
    *pdAccY /= dDelta;


	return DONE;
}
///////////////////////////////////////////////////////////////////////////
/* Private functions */
///////////////////////////////////////////////////////////////////////////
double CNormalPFM::calcLinePotential(int nIntention, double dPosY)
{
    double dPotential = 0.0;

    double dOmega = 2.0;
    double dSigma = 0.3 * DS_LANE_WIDTH;

    double dPositionLeftLine = 0.0;
    double dPositionRightLine = 0.0;

    if( nIntention == KEEPING )
    {
        dPositionLeftLine = DS_CENTERLINE + DS_LANE_WIDTH;
        dPositionRightLine = DS_CENTERLINE;
    }
    else if( nIntention == CHANGING || nIntention == ARRIVAL )
    {
        dPositionLeftLine = DS_CENTERLINE + DS_LANE_WIDTH;
        dPositionRightLine = DS_CENTERLINE - DS_LANE_WIDTH;
    }
    else if( nIntention == ADJUSTMENT )
    {
        dPositionLeftLine = DS_CENTERLINE;
        dPositionRightLine = DS_CENTERLINE - DS_LANE_WIDTH;
    }
    else
    {
        return dPotential;
    }

    double dDistanceFromLeftLine = qAbs( dPosY - dPositionLeftLine );
    double dDistanceFromRightLine = qAbs( dPosY - dPositionRightLine );

    double dPotentialLeft = dOmega * qExp(-(dDistanceFromLeftLine*dDistanceFromLeftLine) / (dSigma * dSigma));
    double dPotentialRight = dOmega * qExp(-(dDistanceFromRightLine*dDistanceFromRightLine) / (dSigma * dSigma));

    dPotential = dPotentialLeft + dPotentialRight;


	return dPotential;
}

double CNormalPFM::calcCarPotential(int nIntention, double dPosX, int nUpdateCounter)
{
    double dPotential = 0.0;
    double dPotentialPreceding = 0.0;
    double dPotentialLead = 0.0;

    double dOmega = CDatabase::GetInstance()->GetDsParameterData( 0 );
    double dSigmaX = CDatabase::GetInstance()->GetDsParameterData( 1 );


    if( nIntention == KEEPING )
    {
        double dPrecedX = 0.0;
        double dPrecedY = 0.0;

        CDatabase::GetInstance()->GetPrecedingTrajectory( nUpdateCounter, &dPrecedX, &dPrecedY );
        double dGap = qAbs( dPrecedX - dPosX );

        dPotentialPreceding = dOmega * qExp(-(dGap * dGap) / (dSigmaX * dSigmaX));
    }
    else if( nIntention == CHANGING || nIntention == ARRIVAL )
    {
        //! Calculate the repulsive potential energy from the preceding vehicle
        double dPrecedX = 0.0;
        double dPrecedY = 0.0;

        CDatabase::GetInstance()->GetPrecedingTrajectory( nUpdateCounter, &dPrecedX, &dPrecedY );
        double dGap = qAbs( dPrecedX - dPosX );

        dPotentialPreceding = dOmega * qExp(-(dGap * dGap) / (dSigmaX * dSigmaX));

        //! Calculate the repulsive potential energy from the lead vehicle
        double dLeadX = 0.0;
        double dLeadY = 0.0;
        CDatabase::GetInstance()->GetLeadTrajectory( nUpdateCounter, &dLeadX, &dLeadY );
        dGap = qAbs( dLeadX - dPosX );

        dPotentialLead = dOmega * qExp(-(dGap * dGap) / (dSigmaX * dSigmaX));
    }
    else if( nIntention == ADJUSTMENT )
    {
        double dLeadX = 0.0;
        double dLeadY = 0.0;
        CDatabase::GetInstance()->GetLeadTrajectory( nUpdateCounter, &dLeadX, &dLeadY );

        double dGap = qAbs( dLeadX - dPosX );

        dPotentialLead = dOmega * qExp(-(dGap * dGap) / (dSigmaX * dSigmaX));
    }
    else
    {
        return dPotential;
    }


    dPotential = dPotentialPreceding + dPotentialLead;


	return dPotential;
}

double CNormalPFM::calcGoalPotential(int nIntention, double dPosX, double dPosY)
{
    double dPotential = 0.0;

    double dWgX = 0.2;
    double dWgY = 0.1;

    double dGoalX = dPosX + 20.0;
    double dGoalY = 0.0;

    double dPotentialX = dWgX * qAbs(dGoalX - dPosX);

    if( nIntention == KEEPING )
    {
        dGoalY = DS_CENTERLINE + 0.5 * DS_LANE_WIDTH;
    }
    else if( nIntention == CHANGING || nIntention == ARRIVAL || nIntention == ADJUSTMENT )
    {
        dGoalY = DS_CENTERLINE - 0.5 * DS_LANE_WIDTH;
    }
    else
    {
        return dPotential;
    }
	
    double dPotentialY = dWgY * qAbs(dGoalY - dPosY);

    dPotential = dPotentialX + dPotentialY;

    return dPotential;
}
