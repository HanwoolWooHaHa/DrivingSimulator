#include "../Include/trainer.h"
#include "../Include/database.h"
#include "../Include/leastsquare.h"
#include "../Library/libpf/potential.h"
#include "../Include/bayesian.h"

#include <QDebug>

CTrainer::CTrainer()
{
	m_nParamCost = 8;

	m_pModel = NULL;
    memset(m_nClassificationResultCounter, 0, sizeof(int) * NUM_CLASS * NUM_CLASS);
    memset(m_dArrayPotential, 0, sizeof(double) * DS_T_MAX);

    CBayesian::GetInstance()->Initialize();
}

CTrainer::~CTrainer()
{
	if (m_pModel != NULL)
		svm_free_and_destroy_model(&m_pModel);
}

void CTrainer::Initialize(int nMode)
{
    switch(nMode)
    {
    case 0:
        memset(m_dArrayPotential, 0, sizeof(double) * DS_T_MAX);
        initializeResultBuffer();
        break;

    case 1:
        memset(m_nClassificationResultCounter, 0, sizeof(int) * NUM_CLASS * NUM_CLASS);
        memset(m_dAccuracy, 0, sizeof(double) * NUM_CLASS);
        qDebug() << "Trainer @ Result counter was initialized... GT = " + QString::number(m_nGroundTruth);
        m_dSuccess = 0;
        m_dFail = 0;
        break;
    }


}

void CTrainer::Train()
{
	//! 1. Set parameters for SVM
	svm_parameter param;
	setParameter(&param);


	//! 2. Make feature vector from data packet
	makeFeatureVectorForTraining();

	//! 3. generate the problem for using Support Vector Machine
	svm_problem problem;
	problem.l = m_pt_list.size();
	problem.y = new double[problem.l];

	int nDataLength = m_pt_list.size();
	if (nDataLength == 0)
	{
		qDebug() << "trainer.cpp @ Training data were not successfully loaded...";
		return;
	}

	svm_node* x_space = new svm_node[(FEATURE_VECTOR_DIMENSION * WINDOW_SIZE + 1) * nDataLength];
	problem.x = new svm_node *[problem.l];

	long i = 0;
	for (std::list <stFeatureVector>::iterator q = m_pt_list.begin(); q != m_pt_list.end(); q++, i++)
	{
		int dim = FEATURE_VECTOR_DIMENSION * WINDOW_SIZE + 1;

		for (int k = 0; k<(FEATURE_VECTOR_DIMENSION * WINDOW_SIZE); k++)
		{
			x_space[dim * i + k].index = k + 1;
			x_space[dim * i + k].value = q->x[k];
		}

		x_space[dim * i + FEATURE_VECTOR_DIMENSION * WINDOW_SIZE].index = -1;

		problem.x[i] = &x_space[dim * i];
		problem.y[i] = q->label;
	}

	if (svm_check_parameter(&problem, &param) != NULL)
	{
		qDebug() << "trainer.cpp @ model has some errors";
		return;
	}


	//! 4. traing the svm model using feature vectors
	qDebug() << "trainer.cpp @ Start to train the svm model!!!!";
	m_pModel = svm_train(&problem, &param);

	qDebug() << "trainer.cpp @ Finished training";

	//! 5. save trained model
    saveModel();


	//svm_free_and_destroy_model(&m_pModel);
	delete[] x_space;
	delete[] problem.x;
	delete[] problem.y;
	free(param.weight_label);
	free(param.weight);
}

void CTrainer::LoadModel()
{
	//! 1. Load trained model from file
	QString modelFile = "Model/Model_K" + QString::number(FEATURE_VECTOR_DIMENSION) + "_C" + QString::number(m_nParamCost) + ".txt";

    std::string cStr = modelFile.toStdString();
	int nLen = cStr.length();
	char* chr = new char[nLen + 1];
	memcpy(chr, cStr.c_str(), nLen + 1);

	m_pModel = svm_load_model(chr);

	delete[] chr;
	if (m_pModel == NULL)
	{
		qDebug() << "mysvm.cpp @ SVM model cannot be loaded";
		return;
	}
}

void CTrainer::Test(int nTick)
{
    if(m_pModel==NULL)
        return;

	if (m_pt_list.size() != 0)
	{
		qDebug() << "trainer.cpp @ List has not been cleared!";
		return;
	}

	if (m_pModel == NULL)
	{
		qDebug() << "trainer.cpp @ SVM model has not been loaded!";
		return;
	}

	//! 2. Make feature vector
	int nCurrentTiral = CDatabase::GetInstance()->GetCurrentTrial();
	makeFeatureVectorForTest(nCurrentTiral, nTick);

	svm_node x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE + 1];
	for (int k = 0; k<FEATURE_VECTOR_DIMENSION * WINDOW_SIZE; k++)
		x[k].index = k + 1;
	x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE].index = -1;

    double* pdProbability = new double[NUM_CLASS];
	double dResult = 0.0;

	for (std::list <stFeatureVector>::iterator q = m_pt_list.begin(); q != m_pt_list.end(); q++)
	{
		for (int k = 0; k<(FEATURE_VECTOR_DIMENSION * WINDOW_SIZE); k++)
			x[k].value = q->x[k];

		dResult = svm_predict_probability(m_pModel, x, pdProbability);

		q->label = (signed char)dResult;
	}

#if defined(PROPOSED)
    int nClassificationResult = judgeClassificationResult((int)dResult, pdProbability);
#else
    int nClassificationResult = (int)dResult;
#endif
    int nGroundTruth = m_nGroundTruth;


	if (nGroundTruth == nClassificationResult)
	{
        m_dSuccess++;
	}
	else
	{
        m_dFail++;
	}

    nGroundTruth--;
    m_dAccuracy[nGroundTruth] = m_dSuccess / (m_dSuccess + m_dFail);

	CDatabase::GetInstance()->SetDsClassificationResult(nCurrentTiral, nTick, nClassificationResult);
    //qDebug() << "trainer.cpp @ t=" << nTick << " :  class = " << nClassificationResult << endl;

    nClassificationResult--;
    m_nClassificationResultCounter[nGroundTruth][nClassificationResult]++;
    m_nClassificationResultBuffer[nClassificationResult]++;

    for(int i=0; i<NUM_CLASS; i++)
    {
        m_dPreProbability[i] = pdProbability[i];
    }

	m_pt_list.clear();

	delete[] pdProbability;
}

double CTrainer::GetAccuracy(int nIndex)
{
	return m_dAccuracy[nIndex];
}

void CTrainer::PrintClassificationCounter(void)
{
    for(int j=0; j<3; j++)
    {
        qDebug() << "trainer.cpp @ [" << m_nGroundTruth-1 << "][" << j << "]=" << m_nClassificationResultCounter[m_nGroundTruth-1][j] << ", ";
    }
    qDebug() << endl << endl;
}

int CTrainer::GetFinalJudge(void)
{
    int nClass = 0;
    int nMax = 0;

    for(int i=0; i<NUM_CLASS; i++)
    {
        int nValue = m_nClassificationResultBuffer[i];
        if(nValue > nMax)
        {
            nMax = nValue;
            nClass = i+1;
        }
    }

    return nClass;
}

///////////////////////////////////////////////////////////////////////////
/* Private member functions */
///////////////////////////////////////////////////////////////////////////
int CTrainer::judgeClassificationResult(int nResult, double* pdProb)
{
    //int nState = CBayesian::GetInstance()->Filter(pdProb, m_dPreProbability);
    int nNumClass = 3;
    int nState = -1;

    double dDelta1 = qAbs(pdProb[0] - pdProb[1]);
    double dDelta2 = qAbs(pdProb[1] - pdProb[2]);
    double dDelta3 = qAbs(pdProb[0] - pdProb[2]);


    if( (dDelta1 < DS_THRESHOLD_FILTER) || (dDelta2 < DS_THRESHOLD_FILTER) || (dDelta3 < DS_THRESHOLD_FILTER) )
    {
        int nSum = 0;
        for(int i=0; i<nNumClass; i++)
        {
            nSum += m_nClassificationResultBuffer[i];
        }

#if defined(MAC)
        double dProb[NUM_CLASS] = {0.0};
#else
        double dProb[nNumClass] = {0.0};
#endif

        for(int i=0; i<nNumClass; i++)
        {
            dProb[i] = pdProb[i] * m_nClassificationResultBuffer[i] / nSum;
        }

        double dMax = 0.0;
        for(int i=0; i<nNumClass; i++)
        {
            if(dProb[i] > dMax)
            {
                dMax = dProb[i];
                nState = i+1;
            }
        }
    }
    else
    {
        nState = nResult;
    }

    return nState;
}

void CTrainer::initializeResultBuffer(void)
{
    for(int i=0; i<NUM_CLASS; i++)
    {
        m_dPreProbability[i] = 1 / NUM_CLASS;
        m_nClassificationResultBuffer[i] = 1;
    }
}

void CTrainer::setParameter(svm_parameter* param)
{
	param->svm_type = C_SVC;
	param->kernel_type = RBF; //LINEAR;//RBF;
	param->degree = 3; //! for poly
	param->gamma = 0.0625; /* for poly/rbf/sigmoid */
	param->coef0 = 1; /* for poly/sigmoid */

	/* these are for training only */
	param->nu = 0.2; /* for NU_SVC, ONE_CLASS, and NU_SVR */
	param->cache_size = 1024; /* in MB */
	param->C = m_nParamCost; /* for C_SVC, EPSILON_SVR, and NU_SVR */
	param->eps = 1e-3; /* stopping criteria */
	param->p = 0.1; /* for EPSILON_SVR */
	param->shrinking = 0; /* use the shrinking heuristics */
	param->probability = 1; /* do probability estimates */
	param->nr_weight = 0; /* for C_SVC */
	param->weight_label = NULL; /* for C_SVC */
	param->weight = NULL; /* for C_SVC */
}

void CTrainer::makeFeatureVectorForTraining(void)
{
	int nNumTrial = CDatabase::GetInstance()->GetNumTrial();

	for (int n = 0; n < nNumTrial; n++)
	{
		int nDataLength = CDatabase::GetInstance()->GetDataInfo(n, DATA_INFO_PACKET_DATA_LENGTH);

#if defined(FILEV)
        CLeastSquare::GetInstance()->Initialize();
        CDatabase::GetInstance()->SetCurrentTrial(n);

        for (int t = 0; t < nDataLength; t++)
        {
            CLeastSquare::GetInstance()->Estimate(t);
            registerVectorToList(n, t);
        }
#else
		for (int t = 0; t < nDataLength; t++)
		{
			registerVectorToList(n, t);
		}
#endif

        CDatabase::GetInstance()->SaveDSdataResult(n, nDataLength);
	}
}

void CTrainer::makeFeatureVectorForTest(int nCurrentTrial, int nTick)
{
	stFeatureVector p;

	for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
	{
		if (nTick >= (WINDOW_SIZE - 1))
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
				double dValue = 0.0;


#if defined(MIKAMI)
                if (k == 0)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue =(dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if (k == 1)
                {
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue =(dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else if (k == 2)
                {
                    double dGap = p.x[WINDOW_SIZE + n];
                    double dRelVel = p.x[n];

                    double dK = 4.0 * 10000000.0 * dRelVel / (dGap*dGap*dGap);

                    if(dK < -1.0)
                    {
                        dValue = 10 * log10(-dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else if(dK > 1.0)
                    {
                        dValue = -10 * log10(dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#elif defined(FILEV)
                dValue = CDatabase::GetInstance()->GetDsParameterData(nCurrentTrial, nTick - n, k);
#elif defined(PROPOSED)
                if(k == 0) // Feature 1 : relative velocity
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if(k == 1) // Feature 2 : relative distance
                {
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else if(k == 2)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_V);

                    double dRelVelX = dPrecedVx - dTargetVx;

                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_X);

                    double dGapPreceding = dPrecedX - dTargetX;

                    double dPotential = CPotential::GetInstance()->Field(dGapPreceding, dRelVelX, 180.0);
                    m_dArrayPotential[nTick] = dPotential;

                    if(nTick >= 602)
                    {
                        double dSum = 0.0;

                        for(int t=0; t<600; t++)
                        {
                            dSum += m_dArrayPotential[nTick - t];
                        }

                        dValue = dSum / 600.0;
                        dValue = (dValue - DS_FEATURE_PE_MEAN) / DS_FEATURE_PE_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature vector has some problem.";
                }
#else
                if (k == 0)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if (k == 1)
                {
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick - n, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#endif

				p.x[k * WINDOW_SIZE + n] = dValue;
			}

		}
		else
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
				double dValue = 0.0;

#if defined(MIKAMI)
				if (k == 0)
				{
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue =(dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
				}
				else if (k == 1)
				{
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue =(dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
				}
                else if (k == 2)
                {
                    double dGap = p.x[WINDOW_SIZE + n];
                    double dRelVel = p.x[n];

                    double dK = 4.0 * 10000000.0 * dRelVel / (dGap*dGap*dGap);

                    if(dK < -1.0)
                    {
                        dValue = 10 * log10(-dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else if(dK > 1.0)
                    {
                        dValue = -10 * log10(dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#elif defined(FILEV)
                dValue = CDatabase::GetInstance()->GetDsParameterData(nCurrentTrial, nTick, k);
#elif defined(PROPOSED)
                if(k == 0)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if(k == 1)
                {
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else if(k == 2)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_V);

                    double dRelVelX = dPrecedVx - dTargetVx;

                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

                    double dGapPreceding = dPrecedX - dTargetX;

                    double dPotential = CPotential::GetInstance()->Field(dGapPreceding, dRelVelX, 180.0);
                    m_dArrayPotential[nTick] = dPotential;

                    if(nTick >= 602)
                    {
                        double dSum = 0.0;

                        for(int t=0; t<600; t++)
                        {
                            dSum += m_dArrayPotential[nTick - t];
                        }

                        dValue = dSum / 600.0;
                        dValue = (dValue - DS_FEATURE_PE_MEAN) / DS_FEATURE_PE_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature vector has some problem.";
                }
#else
                if (k == 0)
                {
                    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
                    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_V);

                    dValue = dPrecedVx - dTargetVx;
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if (k == 1)
                {
                    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
                    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

                    dValue = dPrecedX - dTargetX;
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#endif

				p.x[k * WINDOW_SIZE + n] = dValue;
			}
		}
	}

	// Adding a label at the current time
    p.label = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_CLASS);

	m_pt_list.push_back(p);
}

int CTrainer::registerVectorToList(int nTrial, int nT)
{
	stFeatureVector p;

	for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
	{
		if (nT >= (WINDOW_SIZE - 1))
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
				double dValue = 0.0;

#if defined(MIKAMI)
				if (k == 0)
				{
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_VEL);
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
				}
				else if (k == 1)
				{
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_DST);
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
				}
                else if (k == 2)
                {
                    double dGap = p.x[WINDOW_SIZE + n];
                    double dRelVel = p.x[n];

                    double dK = 4.0 * 10000000.0 * dRelVel / (dGap*dGap*dGap);

                    if(dK < -1.0)
                    {
                        dValue = 10 * log10(-dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else if(dK > 1.0)
                    {
                        dValue = -10 * log10(dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#elif defined(FILEV)
                dValue = CDatabase::GetInstance()->GetDsParameterData(nTrial, nT - n, k);
#elif defined(PROPOSED)
                if(k==0)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_VEL);
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if(k==1)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_DST);
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else if(k==2)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_PE);
                    dValue = (dValue - DS_FEATURE_PE_MEAN) / DS_FEATURE_PE_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some errors.";
                    return FAIL;
                }
#else
                if (k == 0)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_VEL);
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if (k == 1)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT - n, DS_FEATURE_DST);
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#endif

				p.x[k * WINDOW_SIZE + n] = dValue;
			}
				
		}
		else
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
				double dValue = 0.0;

#if defined(MIKAMI)
				if (k == 0)
				{
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_VEL);
                    dValue =(dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
				}
				else if (k == 1)
				{
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_DST);
                    dValue =(dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
				}
                else if (k == 2)
                {
                    double dGap = p.x[WINDOW_SIZE + n];
                    double dRelVel = p.x[n];

                    double dK = 4.0 * 10000000.0 * dRelVel / (dGap*dGap*dGap);

                    if(dK < -1.0)
                    {
                        dValue = 10 * log10(-dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else if(dK > 1.0)
                    {
                        dValue = -10 * log10(dK);
                        dValue =(dValue - DS_FEATURE_RISK_MEAN) / DS_FEATURE_RISK_STD;
                    }
                    else
                    {
                        dValue = 0.0;
                    }
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#elif defined(FILEV)
                dValue = CDatabase::GetInstance()->GetDsParameterData(nTrial, nT, k);
#elif defined(PROPOSED)
                if(k==0)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_VEL);
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if(k==1)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_DST);
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else if(k==2)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_PE);
                    dValue = (dValue - DS_FEATURE_PE_MEAN) / DS_FEATURE_PE_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some errors.";
                    return FAIL;
                }
#else
                if (k == 0)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_VEL);
                    dValue = (dValue - DS_FEATURE_VEL_MEAN) / DS_FEATURE_VEL_STD;
                }
                else if (k == 1)
                {
                    dValue = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_FEATURE_DST);
                    dValue = (dValue - DS_FEATURE_DST_MEAN) / DS_FEATURE_DST_STD;
                }
                else
                {
                    qDebug() << "trainer.cpp @ Setting of feature dimension has some problem.";
                }
#endif

				p.x[k * WINDOW_SIZE + n] = dValue;
			}
		}
	}

	// Adding a label at the current time
    p.label = CDatabase::GetInstance()->GetData(DS, nTrial, nT, DS_CLASS);

	m_pt_list.push_back(p);

	return DONE;
}

void CTrainer::saveModel(void)
{
	QString modelFile = "Model/Model_K" + QString::number(FEATURE_VECTOR_DIMENSION) + "_C" + QString::number(m_nParamCost) + ".txt";
    std::string cStr = modelFile.toStdString();
	int nLen = cStr.length();
	char* chr = new char[nLen + 1];
	memcpy(chr, cStr.c_str(), nLen + 1);

	if (svm_save_model(chr, m_pModel) == 0)
	{
		qDebug() << "mysvm.cpp @ trained model was successfully saved";
	}
	else
	{
		qDebug() << "mysvm.cpp @ It was failed to save the trained model";
	}
}
