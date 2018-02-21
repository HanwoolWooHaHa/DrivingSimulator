#pragma once

class CDatabase;
class CMethod;

/**
* @class	CEstimator
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2016/04/26 \n
* 			    Last revision date: 2016/04/26 HanwoolWoo
* @brief	this class is implemented for the estimation algorithm using sensor information
*/
class CEstimator
{
public:
    enum {SVM=1, HMM, TRAJECTORY};

	static CEstimator* GetInstance(int nMethod)
    {
        static CEstimator* instance = new CEstimator(nMethod);
        return instance;
    }

    ~CEstimator();

	void Estimate(int nTick, int nMode);

private:
	CEstimator(int nMethod);

    int test( int nTick );

	CDatabase* m_pDatabase;
	CMethod* m_pMethod;
};
