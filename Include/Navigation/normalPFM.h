#pragma once

#include "navigationMethod.h"

class CNormalPFM : public CNavigationMethod
{
public:
	CNormalPFM();
	~CNormalPFM();

	virtual void Initialize(void);
    virtual int CalculateAccelerate(int nIntention, double* parrdData, double* pdAccX, double* pdAccY, int nUpdateCounter);

private:
    double calcLinePotential(int nIntention, double dPosY);
    double calcCarPotential(int nIntention, double dPosX, int nUpdateCounter);
    double calcGoalPotential(int nIntention, double dPosX, double dPosY);
};
