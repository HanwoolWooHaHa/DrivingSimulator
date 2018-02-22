/**
* @file	loopManager.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/07/23
* @brief	this file is a manager to make a time step.
*/
#include "../Include/loopManager.h"
#include "../Include/database.h"
#include "../Include/birdView.h"
#include "../Include/trainer.h"
#include "../Include/leastsquare.h"
#include "../Include/majoritycount.h"
#include "../Include/predictor.h"
#include "../Include/evaluation.h"
#include "../Include/estimator.h"
#include "../Include/extractor.h"
#include "../Include/graph.h"

#include <QThread>
#include <QLabel>
#include <QGridLayout>
#include <QDebug>

//#define USE_GRAPH
/*********************************************************************/
CLoopManager::CLoopManager( void ) : DELTA_T(100)
{
    pWindow = new CWindow;
    m_pPredictor = CPredictor::GetInstance();
#ifndef LANE_CHANGE_DETECTION
    m_pEstimator = CEstimator::GetInstance(CEstimator::TRAJECTORY);
#else
    m_pEstimator = CEstimator::GetInstance(CEstimator::SVM);
#endif
    m_pExtractor = CExtractor::GetInstance();
	
    m_pExtractor->Initialize();
	pWindow->Initialize();

	ResetTime();

    m_nDataMode = DRIVING_SIMULATOR_DATA;

    for(int n=0; n<NUM_DRIVER; n++)
    {
        for(int i=0; i<3; i++)
        {
            m_dSuccessCounter[n][i] = 0;
            m_dFailedCounter[n][i] = 0;
        }
    }

    m_nNumFalseAlarm = m_nNumFail = m_nNumSuccess = 0;
    m_dDetectionTime = 0.0;
}

CLoopManager::~CLoopManager()
{
	if (pWindow != NULL)
		delete pWindow;

    if (m_pPredictor != NULL)
        delete m_pPredictor;

    if (m_pEstimator != NULL)
        delete m_pEstimator;

    if (m_pExtractor != NULL)
        delete m_pExtractor;
}

void CLoopManager::ShowWindow( void )
{
	pWindow->show();
}

void CLoopManager::Initialize( void )
{
    CLeastSquare::GetInstance()->Initialize();
}

/*********************************************************************/
/* Public slot functions */
void CLoopManager::DoWork()
{
    while( 1 )
    {
		if( !m_bLoopFlag )
			continue;

        switch(m_nDataMode)
        {
        case DRIVING_SIMULATOR_DATA:
            conductSingleData();
            break;

        case DRIVING_SIMULATOR_ALLDATA:
            conductAllData();
            break;
        }

		QThread::msleep(2);
    }
}

void CLoopManager::TimeoutHandler()
{}

void CLoopManager::DrawTrajectory( int nNoTrajectory, bool bFlag )
{
    pWindow->DrawTrajectory( nNoTrajectory, bFlag );
}

/*********************************************************************/
/* Private slot functions */
bool CLoopManager::checkTestCondition(int nCurrentTrial, int nTick)
{
    int nTrialCheck = nCurrentTrial % 2;
    if(nTrialCheck == 0)
    {
        return false;
    }

    if(nTick < 602 || nTick >= 5000)
    {
        return false;
    }

    double dCenterlineY = -2248.4;
    double dPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_Y);

    if (dPosY <= dCenterlineY)
    {
        return false;
    }

    return true;
}

void CLoopManager::conductSingleData(void)
{
    static int nCurrentTrial = 0;
    static int nDataLength = 0;

    CTrainer::GetInstance()->SetGroundTruth(3);

    if(m_nTick == 0)
    {
        nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
        CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);
    }

#if defined(FILEV)
        //! Estimate parameters used for features
        CLeastSquare::GetInstance()->Estimate(m_nTick);
#endif

    //! Classify driver class
    if(checkTestCondition(nCurrentTrial, m_nTick))
    {
        CTrainer::GetInstance()->Test(m_nTick);
    }


    //! Update a window screen
    pWindow->Update(m_nTick);
    m_nTick++;

    if (m_nTick > nDataLength)
    {
        int nTrialCheck = nCurrentTrial % 2;
        if(nTrialCheck != 0)
        {
            int nFinal = CTrainer::GetInstance()->GetFinalJudge();
            qDebug() << "Trial=" << nCurrentTrial << ": " << nFinal;
        }

        //ResetTime();
        m_nTick = 0;

        // save the data per a trial
        CDatabase::GetInstance()->SaveDSdataResult(nCurrentTrial, nDataLength);

        nCurrentTrial++;
        CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);
        nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
        int nNumTrial = CDatabase::GetInstance()->GetNumTrial();

        CLeastSquare::GetInstance()->Initialize();
        CTrainer::GetInstance()->Initialize(0);

        if (nCurrentTrial >= nNumTrial)
        {
            m_bLoopFlag = false;

            qDebug() << endl << endl << "Classification accuracy = ";
            for(int i=0; i<NUM_CLASS; i++)
                qDebug() << CTrainer::GetInstance()->GetAccuracy(i) << ", ";


            qDebug() << endl << endl;

            CTrainer::GetInstance()->PrintClassificationCounter();
        }
    }
}

void CLoopManager::conductAllData(void)
{
    static int nCurrentTrial = 0;
    static int nDataLength = 0;
    static int nDriverNo = 0;
    static int nStateNo = 0;

    static int nCrossingTime = 0;
    static int nDetectionTime = 0;
    static bool bDetectionFlag = false;

    if(m_nTick == 0)
    {
        if(nCurrentTrial == 0)
        {
            CDatabase::GetInstance()->Initialize();
            CDatabase::GetInstance()->LoadData(DRIVING_SIMULATOR_ALLDATA, DS_FILE_PATH, nDriverNo, nStateNo);

            if(nStateNo == 0 || nStateNo == 1)
            {
                CTrainer::GetInstance()->SetGroundTruth(1);
                CDatabase::GetInstance()->SetGroundTruth(1);
            }
            if(nStateNo == 2 || nStateNo == 3)
            {
                CTrainer::GetInstance()->SetGroundTruth(2);
                CDatabase::GetInstance()->SetGroundTruth(2);
            }
            if(nStateNo == 4 || nStateNo == 5)
            {
                CTrainer::GetInstance()->SetGroundTruth(3);
                CDatabase::GetInstance()->SetGroundTruth(3);
            }

            CTrainer::GetInstance()->Initialize(1);
            m_pExtractor->Initialize();
        }

        nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
        CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);

        //! find the crossing time when the target vehicle crosses the centerline
        nCrossingTime = findCrossingTime( nCurrentTrial, nDataLength );
        nDetectionTime = 0;
        bDetectionFlag = false;
    }


#if defined(DRIVING_STYLE_RECOGNITION)
    //! Classify driver class
    if(checkTestCondition(nCurrentTrial, m_nTick))
    {
        CTrainer::GetInstance()->Test(m_nTick);

        if((m_nTick % MAJORITY_COUNT_SIZE) == 0 )
        {
            int nResult = CMajorityCount::GetInstance()->Count(nCurrentTrial, m_nTick);

            qDebug() << "t=" << m_nTick << " : " << nResult;
        }
    }
#endif


#ifndef LANE_CHANGE_DETECTION
    //! predict the trajectory of the target vehicle
    m_pPredictor->Predict(m_nTick);
#endif
    //! Extract the feature vector for lane-change detection
    m_pExtractor->Extract(m_nTick);

    //! Estimate the driving intention
    m_pEstimator->Estimate(m_nTick, TARGET);

    int nIntention = CDatabase::GetInstance()->GetDrivingIntention();
    //qDebug() << "loopManager.cpp @ t = " << m_nTick << " : intention = " << nIntention << ", Dst = " << CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, m_nTick, FEATURE_PACKET_DISTANCE) << ", Vel = " << CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, m_nTick, FEATURE_PACKET_LAT_VELOCITY) << ", Pot = "  << CDatabase::GetInstance()->GetFeatureData(nCurrentTrial, m_nTick, FEATURE_PACKET_POTENTIAL);

    if (nIntention != DEFAULT)
    {
        m_pPredictor->Predict(m_nTick, nIntention);
        CEvaluation::GetInstance()->CalcTrajectoryPredictionError(m_nTick);

        bDetectionFlag = checkDetectionTime( m_nTick, nIntention, nCrossingTime, bDetectionFlag, &nDetectionTime);
    }



    //! Update a window screen
    pWindow->Update(m_nTick);
    m_nTick++;

    if (m_nTick > nDataLength)
    {
        int nTrialCheck = nCurrentTrial % 2;
        if(nTrialCheck != 0)
        {
            //int nFinal = CTrainer::GetInstance()->GetFinalJudge();
            //qDebug() << "Trial=" << nCurrentTrial << ": " << nFinal;
        }

#if defined(USE_GRAPH)
        int nFileNo = nDriverNo * 100 + nStateNo * 10 + nCurrentTrial;
        pWindow->SaveImage(nFileNo);
#endif

        if( bDetectionFlag )
        {
            CDatabase::GetInstance()->SetDetectionResult( nDriverNo * 100 + nStateNo * 10 + nCurrentTrial, 0, (double)(nDetectionTime)/120.0 );
        }

        CDatabase::GetInstance()->SetDetectionResult( nDriverNo * 100 + nStateNo * 10 + nCurrentTrial, 1, CEvaluation::GetInstance()->GetAvgError(0) );
        CDatabase::GetInstance()->SetDetectionResult( nDriverNo * 100 + nStateNo * 10 + nCurrentTrial, 2, CEvaluation::GetInstance()->GetAvgError(1) );
        CDatabase::GetInstance()->SetDetectionResult( nDriverNo * 100 + nStateNo * 10 + nCurrentTrial, 3, CEvaluation::GetInstance()->GetAvgError(2) );

        //ResetTime();
        m_nTick = 0;

        //CEvaluation::GetInstance()->PrintAvgError();

        // save the data per a trial
        CDatabase::GetInstance()->SaveDSdataResult(nDriverNo, nStateNo, nCurrentTrial, nDataLength);

        nCurrentTrial++;
        CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);
        nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
        int nNumTrial = 10; //CDatabase::GetInstance()->GetNumTrial();

        CLeastSquare::GetInstance()->Initialize();
        CTrainer::GetInstance()->Initialize(0);
        CEvaluation::GetInstance()->InitializeAvgError();
        m_pExtractor->Initialize();

        if (nCurrentTrial >= nNumTrial)
        {
            nCurrentTrial = 0;

            nStateNo++;

            //printDrivingStyleRecognitionResult();
            printDetectionResult( nDriverNo, nStateNo );

            for(int i=0; i<3; i++)
            {
                for(int j=0; j<3; j++)
                {
                    if(i==j)
                    {
                        m_dSuccessCounter[nDriverNo][i] += CTrainer::GetInstance()->GetCounter(i, j);
                    }
                    else
                    {
                        m_dFailedCounter[nDriverNo][i] += CTrainer::GetInstance()->GetCounter(i, j);
                    }
                }
            }

            qDebug() << endl << endl;
        }

        if( nStateNo >= NUM_STATE )
        {
            nCurrentTrial = 0;
            nStateNo = 0;

            nDriverNo++;
        }

        if(nDriverNo >= NUM_DRIVER)
        {
            double dSumSuccess = 0;
            double dSumFail = 0;

            for(int n=0; n<NUM_DRIVER; n++)
            {
                qDebug() << "Loop @ Driver No." << n;

                for(int i=0; i<3; i++)
                {
                    switch(i)
                    {
                    case 0:
                        qDebug() << "Loop @ Cautious";
                        break;
                    case 1:
                        qDebug() << "Loop @ Normal";
                        break;
                    case 2:
                        qDebug() << "Loop @ Hurry";
                        break;
                    }

                    qDebug() << "Loop @ Success=" << m_dSuccessCounter[n][i] << ", Fail=" << m_dFailedCounter[n][i];
                    qDebug() << "Loop @ Accuracy = " << m_dSuccessCounter[n][i] / (m_dSuccessCounter[n][i]+m_dFailedCounter[n][i]) * 100.0;

                    dSumSuccess += m_dSuccessCounter[n][i];
                    dSumFail += m_dFailedCounter[n][i];

                    qDebug() << endl;
                }
            }

            qDebug() << "Loop @ Accuracy = " << dSumSuccess / (dSumSuccess+dSumFail) * 100.0;

            //CMajorityCount::GetInstance()->PrintAccuracy();

            CDatabase::GetInstance()->SaveDetectionResult();

            m_bLoopFlag = false;
        }
    }
}

void CLoopManager::printDrivingStyleRecognitionResult( void )
{
    qDebug() << endl << endl << "Classification accuracy = ";
    for(int i=0; i<NUM_CLASS; i++)
        qDebug() << CTrainer::GetInstance()->GetAccuracy(i) << ", ";

    CTrainer::GetInstance()->PrintClassificationCounter();
}

int CLoopManager::findCrossingTime( int nCurrentTrial, int nDataLength )
{
    int nCrossingTime = -1;

    for(int t=0; t<nDataLength; t++)
    {
        double dPosY = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, t, DS_OWN_Y);

        if(dPosY <= DS_CENTERLINE)
        {
            nCrossingTime = t;
            break;
        }
    }

    return nCrossingTime;
}

bool CLoopManager::checkDetectionTime( int nTick, int nIntention, int nCrossingTime, bool bDetectionFlag, int* pnDetectionTime )
{
    bool bFlag = false;
    int nDetectionTime = INT_MAX;

    //! if lane changing was already detected
    if( bDetectionFlag )
        return true;


    if( nCrossingTime == -1 || nCrossingTime == 0 )
    {
        *pnDetectionTime = INT_MAX;
        return false;
    }


    if( nIntention == CHANGING )
    {
        nDetectionTime = nCrossingTime - nTick;
        bFlag = true;

        qDebug() << "CLoopManager.cpp @ t = " << nTick << " : Lane changing was detected, TIME = " << (double)(nDetectionTime)/120.0;

        if( nDetectionTime <= 0 )
            m_nNumFail++;
        else
        {
            if( nDetectionTime >= ( DETECTION_TIME_LIMIT * 120) )
                m_nNumFalseAlarm++;
            else
                m_nNumSuccess++;
        }

        *pnDetectionTime = nDetectionTime;
        m_dDetectionTime += nDetectionTime;
    }

    return bFlag;
}

void CLoopManager::printDetectionResult( int nDriverNo, int nStateNo )
{
    m_dDetectionTime = m_dDetectionTime / ( 120.0 * (double)( m_nNumFail + m_nNumFalseAlarm + m_nNumSuccess ) );
    qDebug() << "CLoopManager.cpp @ Driver No." << nDriverNo << ", State No." << nStateNo << " : False alarm=" << m_nNumFalseAlarm << ", Fail=" << m_nNumFail << ", Success=" << m_nNumSuccess <<", DetectionTime=" << m_dDetectionTime;

    m_nNumSuccess = m_nNumFalseAlarm = m_nNumFail = 0;
    m_dDetectionTime = 0.0;
}

/*********************************************************************/
/* Class CWindow */
/*********************************************************************/
CWindow::CWindow()
{
    setWindowTitle(tr("Simulation viewer"));

    m_pBirdView = new CBirdView();
#if defined(USE_GRAPH)
    m_pGraph = new CGraph();
    m_pGraph->show();
#endif

    //QLabel *birdViewLabel = new QLabel(tr("TOP VIEW"));
    //birdViewLabel->setAlignment(Qt::AlignHCenter);

    QGridLayout *layout = new QGridLayout;
    
	layout->addWidget(m_pBirdView,0,1);
    //layout->addWidget(birdViewLabel,1,1);

    setLayout(layout);
}

void CWindow::Initialize( void )
{
    m_pBirdView->Initialize();
}

void CWindow::Update( int tick )
{
    m_pBirdView->Update( tick );
#if defined(USE_GRAPH)
    m_pGraph->Update( tick );
#endif
}

void CWindow::SaveImage(int nTrialNo)
{
    m_pGraph->SaveImage(nTrialNo);
}

void CWindow::DrawTrajectory(int nNoTrajectory, bool bFlag)
{
    m_pBirdView->DrawTrajectory( nNoTrajectory, bFlag );
}
