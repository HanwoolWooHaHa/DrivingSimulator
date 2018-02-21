#pragma once

class CNavigationMethod
{
public:
	virtual void Initialize(void) = 0;
    virtual int CalculateAccelerate(int nIntention, double* parrdData, double* pdAccX, double* pdAccY) = 0;
};
