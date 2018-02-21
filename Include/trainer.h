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
    int GetCounter(int nRow, int nColumn) { return m_nClassificationResultCounter[nRow][nColumn]; }
    void PrintClassificationCounter(void);
    void Initialize(int nMode);
    int GetFinalJudge(void);

    void SetGroundTruth(int nLabel) { m_nGroundTruth = nLabel; }

private:
	CTrainer();

	void setParameter(svm_parameter* param);
	void makeFeatureVectorForTraining(void);
	void makeFeatureVectorForTest(int nCurrentTrial, int nTick);
    void saveModel(void);
	int registerVectorToList(int nIndex, int nT);
    int judgeClassificationResult(int nResult, double* pdProb);
	void initializeResultBuffer(void);

	std::list<stFeatureVector> m_pt_list;
	int m_nParamCost;
    int m_nGroundTruth;
    double m_dSuccess;
    double m_dFail;
	svm_model* m_pModel;
    int m_nClassificationResultBuffer[NUM_CLASS];
    int m_nClassificationResultCounter[NUM_CLASS][NUM_CLASS];
    double m_dAccuracy[NUM_CLASS];
    double m_dArrayPotential[DS_T_MAX];
    double m_dPreProbability[NUM_STATE];
};
