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

#include <QThread>
#include <QLabel>
#include <QGridLayout>
#include <QDebug>
/*********************************************************************/
CLoopManager::CLoopManager( void ) : DELTA_T(100)
{
    pWindow = new CWindow;
	
	pWindow->Initialize();

	ResetTime();
}

CLoopManager::~CLoopManager()
{
	if (pWindow != NULL)
		delete pWindow;
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
	static int nCurrentTrial = 0;
	int nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
	CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);

    while( 1 )
    {
		if( !m_bLoopFlag )
			continue;

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
			//ResetTime();
			m_nTick = 0;

			// save the data per a trial
            CDatabase::GetInstance()->SaveDSdataResult(nCurrentTrial, nDataLength);

			nCurrentTrial++;
			CDatabase::GetInstance()->SetCurrentTrial(nCurrentTrial);
			nDataLength = CDatabase::GetInstance()->GetDataInfo(nCurrentTrial, DATA_INFO_PACKET_DATA_LENGTH);
			int nNumTrial = CDatabase::GetInstance()->GetNumTrial();

            CLeastSquare::GetInstance()->Initialize();
            CTrainer::GetInstance()->Initialize();

			if (nCurrentTrial >= nNumTrial)
			{
				m_bLoopFlag = false;
				qDebug() << endl << endl << "Classification accuracy = " << CTrainer::GetInstance()->GetAccuracy(0) << ", " << CTrainer::GetInstance()->GetAccuracy(1) << ", " << CTrainer::GetInstance()->GetAccuracy(2) << ", " << CTrainer::GetInstance()->GetAccuracy(3) << endl ;

                qDebug() << endl << endl;

                CTrainer::GetInstance()->PrintClassificationCounter();
			}
		}
		

		QThread::msleep(2);
    }
}

void CLoopManager::TimeoutHandler()
{}

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


/*********************************************************************/
/* Class CWindow */
/*********************************************************************/
CWindow::CWindow()
{
    setWindowTitle(tr("Simulation viewer"));

    m_pBirdView = new CBirdView();

    QLabel *birdViewLabel = new QLabel(tr("TOP VIEW"));
    birdViewLabel->setAlignment(Qt::AlignHCenter);

    QGridLayout *layout = new QGridLayout;
    
	layout->addWidget(m_pBirdView,0,1);
    layout->addWidget(birdViewLabel,1,1);

    setLayout(layout);
}

void CWindow::Initialize( void )
{
    m_pBirdView->Initialize();
}

void CWindow::Update( int tick )
{
    m_pBirdView->Update( tick );
}
