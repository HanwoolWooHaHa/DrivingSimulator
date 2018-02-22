/**
* @file	mysvm.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/05/12
* @brief	this file is for the estimation using a support vector machine
*/
#include "../Include/mySVM.h"
#include "../Include/database.h"

#include <qdebug.h>
#include <qfile.h>
#include <qtextstream.h>
#include <time.h>
#include <float.h>
#include <cmath>



CMySvm::CMySvm()
{
	m_pModel = NULL;
	memset(m_dProbabiliy, 0, sizeof(double)* NUM_CLASS * T_MAX);

	m_nParamCost = 8;

    loadModel();

    if(svm_check_probability_model(m_pModel))
        qDebug() << "mySVM.cpp @ svm model was successfully loaded...";
}

CMySvm::~CMySvm()
{
	svm_free_and_destroy_model(&m_pModel);
	delete m_pModel;
}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
void CMySvm::Train(int nType)
{
    qDebug() << nType;
}

bool CMySvm::Test(int nTick, int nMode)
{
	bool bEstimationResult = false;
	double dClassificationResult = -1;

	//  1. make the packet data for the use to SVM
	makePacketForSVM(nTick, nMode);

	svm_node x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE + 1];
	for (int k = 0; k<FEATURE_VECTOR_DIMENSION * WINDOW_SIZE; k++)
		x[k].index = k + 1;
	x[FEATURE_VECTOR_DIMENSION * WINDOW_SIZE].index = -1;

    double* pdProbability = new double[NUM_CLASS];

	for (std::list <stFeatureVector>::iterator q = m_feature_vector_list.begin(); q != m_feature_vector_list.end(); q++)
	{
		for (int k = 0; k<(FEATURE_VECTOR_DIMENSION * WINDOW_SIZE); k++)
			x[k].value = q->x[k];

        dClassificationResult = svm_predict_probability(m_pModel, x, pdProbability);

		q->label = (signed char)dClassificationResult;
        q->prob = pdProbability[1];

		CDatabase::GetInstance()->SetEstimatedResult(nTick, dClassificationResult);
        CDatabase::GetInstance()->SetDrivingIntention( (int)dClassificationResult );

        //qDebug() << "t = " << nTick << ": 1=" << pdProbability[0] << ", 2=" << pdProbability[1] << ", 3=" << pdProbability[2] << ", 4=" << pdProbability[3] << ",   int=" << (int)dClassificationResult;

        for(int i=0; i<NUM_CLASS; i++)
        {
            CDatabase::GetInstance()->SetIntentionProbability(i, pdProbability[i]);
        }
    }

	delete[] pdProbability;

	return bEstimationResult;
}
///////////////////////////////////////////////////////////////////////////
/* Private member functions */
///////////////////////////////////////////////////////////////////////////
void CMySvm::setParameter(svm_parameter* param)
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

void CMySvm::makePacketForSVM(int nTick, int nMode)
{
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dFeatureDistance = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_DISTANCE);
    double dFeatureLateralVelocity = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_LAT_VELOCITY);
    double dFeaturePotential = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick, FEATURE_PACKET_POTENTIAL);

    if (std::isnan(dFeatureDistance) || std::isnan(dFeatureLateralVelocity) || std::isnan(dFeaturePotential))
		return;

    m_feature_vector_list.clear();

	stFeatureVector p;

	if (nTick >= (WINDOW_SIZE-1))
	{

		bool bNandFlag = false;
		for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
                double dValue = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick - n, k);

                if (std::isnan(dValue))
				{
					bNandFlag = true;
					break;
				}
			}

			if (bNandFlag)
				break;
		}

        if (bNandFlag)
		{
			for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
			{
				for (int n = 0; n < WINDOW_SIZE; n++)
				{
                    p.x[k * WINDOW_SIZE + n] = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick, k);
				}
			}
		}
		else
		{
			for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
			{
				for (int n = 0; n < WINDOW_SIZE; n++)
				{
                    p.x[k * WINDOW_SIZE + n] = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick - n, k);
				}
			}
		}
	}
    else
	{
		for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
		{
			for (int n = 0; n < WINDOW_SIZE; n++)
			{
                p.x[k * WINDOW_SIZE + n] = CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, nTick, k);
			}
		}
	}


	m_feature_vector_list.push_back(p);
}

double CMySvm::bayesianFilter(int nTime, double dResult, double* dProb)
{
	if (nTime == 0)
	{
		for (int i = 0; i<NUM_CLASS; i++)
			m_dProbabiliy[i][nTime] = dProb[i];
	}
	else
	{
		for (int i = 0; i<NUM_CLASS; i++)
		{
			double dSum = 0.0;
			for (int j = 0; j<NUM_CLASS; j++)
			{
//				dSum += CDatabase::GetInstance()->GetStateTransitionMatrix(j, i) * m_dProbabiliy[j][nTime - 1];
			}
			m_dProbabiliy[i][nTime] = dProb[i] * dSum;
		}
	}

	double dProbMax = DBL_MIN;
	for (int i = 0; i < NUM_CLASS; i++)
	{
		if (m_dProbabiliy[i][nTime] > dProbMax)
		{
			dProbMax = m_dProbabiliy[i][nTime];
			dResult = i + 1;
		}
	}

	return dResult;
}

void CMySvm::saveModel( void )
{
    //QString modelFile = "Model/Model_K" + QString::number(FEATURE_VECTOR_DIMENSION) + "_C" + QString::number(m_nParamCost) + ".txt";
    std::string cStr =0;// modelFile.toLocal8Bit();
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

void CMySvm::loadModel( void )
{
    QString modelFile = "Model/160324_Proposed(SVM).txt";

    std::string cStr = modelFile.toStdString();
    int nLen = modelFile.length();
    char* chr = new char[nLen + 1];
    memcpy(chr, cStr.c_str(), nLen + 1);

    m_pModel = svm_load_model(chr);

    delete[] chr;
}
