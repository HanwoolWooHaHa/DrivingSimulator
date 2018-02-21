#pragma once

#include "method.h"
#include "../Library/libsvm-3.20/svm.h"
#include <list>
#include "defines.h"

typedef struct
{
	double x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE];
	signed char label;
	double prob;
}stFeatureVector;

/**
* @class	CMySvm
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2016/05/12 \n
* 			    Last revision date: 2016/05/12 HanwoolWoo
* @brief	this class is the implementation of Support Vector Machine using LibSVM
*/
class CMySvm : public CMethod
{
public:
	CMySvm();
	~CMySvm();

	virtual void Train(int nType);
	virtual bool Test(int nTick, int nMode);

private:
	void setParameter(svm_parameter* param);
	
    void loadModel( void );
    void saveModel( void );
	double bayesianFilter(int nTime, double dResult, double* dProb);

    void makePacketForSVM(int nTick, int nMode);

    std::list<stFeatureVector> m_feature_vector_list;

	svm_model* m_pModel;

	int m_nParamCost;

    double m_dProbabiliy[NUM_CLASS][T_MAX];
};
