#pragma once

class CNavigationMethod
{
public:
	virtual void Initialize(void) = 0;
	virtual int CalculateAccelerate(int nTick, bool bLaneChangingFlag, double* pdDataArray, double* pdAccX, double* pdAccY) = 0;
};