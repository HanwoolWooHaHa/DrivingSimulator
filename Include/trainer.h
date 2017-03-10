#pragma once

#include "../Include/defines.h"
#include "../Library/libsvm-3.20/svm.h"
#include <list>

typedef struct
{
	double x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE];
	signed char label;
	double prob;
}stFeatureVector;

class CTrainer
{
public:
	static CTrainer* GetInstance()
	{
		static CTrainer* instance = new CTrainer();
		return instance;
	}

	~CTrainer();

	void Train();
	void LoadModel();
	void Test(int nTick);
	double GetAccuracy(int nIndex);
    void PrintClassificationCounter(void);
    void Initialize(void);

private:
	CTrainer();

	void setParameter(svm_parameter* param);
	void makeFeatureVector(void);
	void makeFeatureVectorForTest(int nCurrentTrial, int nTick);
    void saveModel(void);
	int registerVectorToList(int nIndex, int nT);
	int judgeClassificationResult(int nTick, double dResult, double* pdProbability);
	void initializeResultBuffer(void);

	std::list<stFeatureVector> m_pt_list;
	int m_nParamCost;
	svm_model* m_pModel;
    int m_nClassificationResultBuffer[NUM_STATE];
    int m_nClassificationResultCounter[NUM_STATE][NUM_STATE];
	double m_dAccuracy[4];
    double m_dArrayPotential[DS_T_MAX];
};
