#pragma once

/**
* @class	CPredictor
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2018/02/20
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
    bool Predict( int nTick, int nIntention );

private:
	CPredictor( void );

	CNavigationMethod* m_pNaviMethod;

    void update(double* arrData, double dTgtAccX, double dTgtAccY, double dDelta);
	void calculateAverageVelocity(int nTick, double* pdAdjData);
};
