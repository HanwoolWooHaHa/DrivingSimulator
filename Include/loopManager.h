#pragma once

#include <QWidget>

class CDriverView;
class CBirdView;
class CEstimator;
class CExtractor;
class CPredictor;
class CGraph;

/**
* @class	CWindow
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2015/12/23 \n
* 			    Last revision date: 2015/12/23 HanwoolWoo
* @brief	
*/
class CWindow : public QWidget
{
    Q_OBJECT

public:
    CWindow();

    void Initialize( void );
    void Update( int tick );
    void SaveImage( int nTrialNo );
    void DrawTrajectory( int nNoTrajectory, bool bFlag );

private:
    CDriverView* m_pDriverView;
	CBirdView* m_pBirdView;
    CGraph* m_pGraph;
};

/**
* @class	CLoopManager
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2015/12/23 \n
* 			    Last revision date: 2015/12/23 HanwoolWoo
* @brief	
*/
class CLoopManager : public QObject
{
	Q_OBJECT

public:
    explicit CLoopManager( void );
	~CLoopManager();

	void ResetTime( void ) { m_nTick = 0; m_bLoopFlag = false; }

	void ShowWindow( void );

	void SetLoopFlag( bool flag ) { m_bLoopFlag = flag; }
	bool GetLoopFlag( void ) { return m_bLoopFlag; }

	void SetAutoFlag(bool bFlag) { m_bAutoFlag = bFlag; }

    void SetDataMode(int nDataMode) { m_nDataMode = nDataMode; }

    void Initialize( void );

    void DrawTrajectory( int nNoTrajectory, bool bFlag );

	CWindow* pWindow;

private:
	CEstimator* m_pEstimator;
	CExtractor* m_pExtractor;
	CPredictor* m_pPredictor;

	void conductSingleData(void);
    void conductAllData(void);
	void printAutomationResult(int nNumData);
    bool checkTestCondition(int nCurrentTrial, int nTick);
    void printDrivingStyleRecognitionResult( void );
    void printDetectionResult( int nDriverNo, int nStateNo );
    int findCrossingTime( int nCurrentTrial, int nDataLength );
    bool checkDetectionTime( int nTick, int nIntention, int nCrossingTime, bool bDetectionFlag, int* pnDetectionTime );

public slots:
    void DoWork();
    void TimeoutHandler();

private:
	const int DELTA_T;
	int m_nTick;
	bool m_bLoopFlag;
	bool m_bAutoFlag;
	int m_arrnDetectionTime[400][2];
    double m_dSuccessCounter[10][3];
    double m_dFailedCounter[10][3];
    int m_nDataMode;
    int m_nNumFalseAlarm;
    int m_nNumFail;
    int m_nNumSuccess;
    double m_dDetectionTime;
};
