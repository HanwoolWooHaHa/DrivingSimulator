#pragma once

/**
* @class	CPredictor
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2016/08/04
* @brief
*/

class CNavigationMethod;

class CPredictor
{
public:
	static CPredictor* GetInstance( void )
	{
		static CPredictor* instance = new CPredictor();
		return instance;
	}

	~CPredictor();

	bool Predict( int nTick );

private:
	CPredictor( void );

	CNavigationMethod* m_pNaviMethod;

	void setGoalPosition(bool bLaneChangingFlag, double dTgtPosX, double dTgtPosY, double* pdGoalPosX, double* pdGoalPosY);
	void update(bool bLaneChangingFlag, double* arrData, double dTgtAccX, double dTgtAccY, double dDelta);
	void calculateAverageVelocity(int nTick, double* pdAdjData);
};