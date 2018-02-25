/**
* @file	database.cpp
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/01/16
* @brief	this file is for the management of data files
*/
#include "../Include/database.h"
#include "../Library/libpf/potential.h"

#include <qfile.h>
#include <qtextstream.h>
#include <qdebug.h>
#if defined(MAC)
#include <math.h>
#endif

static QString DRIVER[10] =
{
    "fujihara_", //0
    "fukuba_", //1
    "fushima_", //2
    "hisamitsu_", //3
    "kuroda_", //4
    "kusumoto_", //5
    "matsuoka_", //6
    "sugano_", //7
    "takahashi_", //8
    "uramoto_" //9
};

static QString STATE[6] =
{
    "cautious_1-10",
    "cautious_11-20",
    "normal_1-10",
    "normal_11-20",
    "hurry_1-10",
    "hurry_11-20"
};

CDatabase::CDatabase()
{
    Initialize();

    memset(m_dDetectionResult, 0, sizeof(double) * 1000 * 4);
}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
/// \brief CDatabase::Initialize
///
void CDatabase::Initialize( void )
{
    m_nDataLength = 0;
	m_nGroundTruth = 0;

    memset(m_dDrivingSimulatorData, 0, sizeof(double) * DS_NUM_TRIAL * DS_T_MAX * DS_NUM_COLUMN);
	m_nTrial = 0;
	m_nCurrentTrial = 0;

    m_bShowOthers = true;
    m_nDrivingIntention = 0;
	m_bCollisionFlag = false;
    m_dAcceptanceProbability = 0.0;
    m_dCriticalLeadGap = 0.0;
    m_dSumCriticalLeadGap = 0.0;

    memset(m_dPredictedTrajectory, 0, sizeof(double) * 100 *2);
    memset(m_dIntentionProbability, 0, sizeof(double) * NUM_CLASS);
    memset(m_nEstimatedResultData, 0, sizeof(int) * DS_T_MAX);
    memset(m_dFeatureData, 0, sizeof(double) * DS_NUM_TRIAL * DS_T_MAX * FEATURE_VECTOR_DIMENSION);
}

void CDatabase::SetCollisionFlag(bool bFlag)
{
	m_bCollisionFlag = bFlag;
}

bool CDatabase::GetCollisionFlag(void)
{
	return m_bCollisionFlag;
}

void CDatabase::SetFlagShowOthers( bool flag )
{
    m_bShowOthers = flag;
}

bool CDatabase::GetFlagShowOthers( void )
{
    return m_bShowOthers;
}

void CDatabase::SetDrivingIntention( int nIntention )
{
    m_nDrivingIntention = nIntention;
}

int CDatabase::GetDrivingIntention( void )
{
    return m_nDrivingIntention;
}

void CDatabase::SetDataInfo(int nVehicleIndex, int nColumn, int nValue)
{
	m_nDataInfo[nVehicleIndex][nColumn] = nValue;
}

void CDatabase::SetPredictedTrajectory(int nIndex, double dPosX, double dPosY)
{
    if(nIndex >= 100)
    {
        qDebug() << "Database.cpp @ Index of trajectory is over the limit";
        return;
    }

    m_dPredictedTrajectory[nIndex][0] = dPosX;
    m_dPredictedTrajectory[nIndex][1] = dPosY;
}

void CDatabase::GetPredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY)
{
    *pdPosX = m_dPredictedTrajectory[nIndex][0];
    *pdPosY = m_dPredictedTrajectory[nIndex][1];
}

void CDatabase::SetPrecedingTrajectory(int nIndex, double dPosX, double dPosY)
{
    if(nIndex >= 100)
    {
        qDebug() << "Database.cpp @ Index of trajectory is over the limit";
        return;
    }

    m_dPrecedingTrajectory[nIndex][0] = dPosX;
    m_dPrecedingTrajectory[nIndex][1] = dPosY;
}

void CDatabase::GetPrecedingTrajectory(int nIndex, double* pdPosX, double* pdPosY)
{
    *pdPosX = m_dPrecedingTrajectory[nIndex][0];
    *pdPosY = m_dPrecedingTrajectory[nIndex][1];
}

void CDatabase::SetCriticalLeadGap( double dValue )
{
    static int nCounter = 0;

    if( dValue == 0.0 )
    {
        m_dSumCriticalLeadGap = 0.0;
        nCounter = 0;
        m_dCriticalLeadGap = 0.0;

        return;
    }

    nCounter++;
    m_dSumCriticalLeadGap += dValue;
    m_dCriticalLeadGap = m_dSumCriticalLeadGap / (double)nCounter;
}

double CDatabase::GetCriticalLeadGap( void )
{
    return m_dCriticalLeadGap;
}

void CDatabase::SetLeadTrajectory(int nIndex, double dPosX, double dPosY)
{
    if(nIndex >= 100)
    {
        qDebug() << "Database.cpp @ Index of trajectory is over the limit";
        return;
    }

    m_dLeadTrajectory[nIndex][0] = dPosX;
    m_dLeadTrajectory[nIndex][1] = dPosY;
}

void CDatabase::GetLeadTrajectory(int nIndex, double* pdPosX, double* pdPosY)
{
    *pdPosX = m_dLeadTrajectory[nIndex][0];
    *pdPosY = m_dLeadTrajectory[nIndex][1];
}

int CDatabase::GetDataInfo(int nVehicleIndex, int nColumn)
{
	return m_nDataInfo[nVehicleIndex][nColumn];
}

void CDatabase::SetDetectionResult( int nTrialNo, int nColumn, double dValue )
{
    m_dDetectionResult[nTrialNo][nColumn] = dValue;
}

/// \brief CDatabase::LoadData
/// \param nDataMode
/// \return int
///
int CDatabase::LoadData(int nDataMode, QString fileName)
{
	int nReturn = FAIL;

	switch (nDataMode)
	{
	case TRAFFIC_DATA:
		break;

	case DRIVING_SIMULATOR_DATA:
        nReturn = loadDSData(fileName);
        break;

    case DRIVING_SIMULATOR_THREE:
        nReturn = loadDSThreeData();
        break;
	}

	return nReturn;
}

int CDatabase::LoadData(int nDataMode, QString fileName, int nDriverNo, int nStateNo)
{
    int nReturn = FAIL;

    if(nDataMode != DRIVING_SIMULATOR_ALLDATA)
    {
        return FAIL;
    }

    nReturn = loadDSAllData(fileName, nDriverNo, nStateNo);

    return nReturn;
}

///
/// \brief CDatabase::GetDataLength
/// \return
///
int CDatabase::GetDataLength(void)
{
    return m_nDataLength;
}

int CDatabase::GetGroundTruth(void)
{
	return m_nGroundTruth;
}

void CDatabase::SetGroundTruth(int nValue)
{
    m_nGroundTruth = nValue;
}

///
/// \brief CDatabase::GetData
/// \param nVehicleType
/// \param nVehicleIndex
/// \param nTime
/// \param nColumn
/// \return
///
double CDatabase::GetData(int nVehicleType, int nVehicleIndex, int nTime, int nColumn)
{
    switch(nVehicleType)
    {
	case DS:
		return m_dDrivingSimulatorData[nVehicleIndex][nTime][nColumn];
    }

    return FAIL;
}

int CDatabase::GetNumTrial(void)
{
	return m_nTrial;
}

void CDatabase::SetDsClassificationResult(int nCurrentTrial, int nTick, int nResult)
{
    m_dDrivingSimulatorData[nCurrentTrial][nTick][DS_CLASS] = (double)nResult;
}

int CDatabase::GetDsClassificationResult(int nCurrentTrial, int nTick)
{
    return (int)m_dDrivingSimulatorData[nCurrentTrial][nTick][DS_CLASS];
}

int CDatabase::SaveDataPerTrial(void)
{
    int nNumTrial = m_nTrial;

    for(int n=0; n<nNumTrial; n++)
    {
        QString fileName = "../Log/DS/TrialNo";
        fileName += QString::number(n) + ".csv";

        QFile* fp = new QFile(fileName);
        if (!fp->open(QIODevice::WriteOnly))
        {
            return FAIL;
        }

        QTextStream* out = new QTextStream(fp);

        int nDataLength = GetDataInfo(n, DATA_INFO_PACKET_DATA_LENGTH);

        for(int t=0; t<nDataLength; t++)
        {
            double dVel1 = m_dDrivingSimulatorData[n][t][18];
            double dVel2 = m_dDrivingSimulatorData[n][t][23];

            double dDelta = dVel1 - dVel2;

            if(dDelta != 0.0)
                continue;

            for(int k=0; k<DS_NUM_COLUMN; k++)
            {
                *out << m_dDrivingSimulatorData[n][t][k] << ",";
            }

            *out << endl;
        }

        fp->close();

        delete fp;
        delete out;
    }

    return DONE;
}

int CDatabase::SaveDSdataResult(int nDriverNo, int nState, int nTrial, int nDataLength)
{
    if(nTrial >= 10)
    {
        return FAIL;
    }

    int nFileNo = nDriverNo * 100 + nState * 10 + nTrial;

    QString fileName = "../Log/DS/TrialNo";
    fileName += QString::number(nFileNo) + ".csv";

    QFile* fp = new QFile(fileName);
    if (!fp->open(QIODevice::WriteOnly))
    {
        return FAIL;
    }

    QTextStream* out = new QTextStream(fp);

    double dCenterlineY = -2248.4;
    double dArrayPotential[DS_T_MAX] = { 0.0 };

    for (int t = 0; t<nDataLength; t++)
    {
        double dPosX = m_dDrivingSimulatorData[nTrial][t][DS_OWN_X];
        double dPosY = m_dDrivingSimulatorData[nTrial][t][DS_OWN_Y];
        double dVelX = m_dDrivingSimulatorData[nTrial][t][DS_OWN_V];

        double dPrecedingPosX = m_dDrivingSimulatorData[nTrial][t][DS_PRECED_X];
        double dPrecedingVelX = m_dDrivingSimulatorData[nTrial][t][DS_PRECED_V];

        double dGapPreceding = dPrecedingPosX - dPosX;
        double dRelVelX = dPrecedingVelX - dVelX;

        int nProcess = -1;
        if (dPosY > dCenterlineY)
            nProcess = 1;
        else
            nProcess = 2;

        if (nProcess == 1 && (dPosX > dPrecedingPosX))
            nProcess = 3;

        int nClassificationResult = m_dDrivingSimulatorData[nTrial][t][DS_CLASS];

        if(FEATURE_VECTOR_DIMENSION == 2)
        {
            *out << t << "," << dPosX << "," << dPosY << "," << nProcess << "," << dGapPreceding << "," << dRelVelX << "," << "," << nClassificationResult << endl;
        }
        else if(FEATURE_VECTOR_DIMENSION == 3)
        {
            double dKdB = 0.0;
            double dK = 4.0 * 10000000.0 * dRelVelX / (dGapPreceding * dGapPreceding * dGapPreceding);

            if(dK < -1.0)
            {
                dKdB = 10 * log10(-dK);
            }
            else if(dK > 1.0)
            {
                dKdB = -10 * log10(dK);
            }
            else
            {
                dKdB = 0.0;
            }

            double dPotential = CPotential::GetInstance()->Field(dGapPreceding, dRelVelX, 180.0);
            dArrayPotential[t] = dPotential;

            double dAvg = 0.0;

            if(t >= 602)
            {
                double dSum = 0.0;

                for(int k=0; k<600; k++)
                {
                    dSum += dArrayPotential[t-k];
                }

                dAvg = dSum / 600.0;
            }

            if(dAvg != 0.0 && nProcess == 1 && t < 5000)
                *out << t << "," << dPosX << "," << dPosY << "," << nProcess << "," << "," << dGapPreceding << "," << dRelVelX << "," << dKdB << "," << "," << dPotential << ", " << dAvg << "," << "," << nClassificationResult << endl;
        }


    }

    fp->close();

    delete fp;
    delete out;

    return DONE;
}

int CDatabase::SaveDSdataResult(int nTrial, int nDataLength)
{
    QString fileName = "../Log/DS/TrialNo";
	fileName += QString::number(nTrial) + ".csv";

	QFile* fp = new QFile(fileName);
	if (!fp->open(QIODevice::WriteOnly))
	{
		return FAIL;
	}

	QTextStream* out = new QTextStream(fp);

    double dCenterlineY = -2248.4;
    double dArrayPotential[DS_T_MAX] = { 0.0 };

	for (int t = 0; t<nDataLength; t++)
	{
        double dPosX = m_dDrivingSimulatorData[nTrial][t][DS_OWN_X];
        double dPosY = m_dDrivingSimulatorData[nTrial][t][DS_OWN_Y];
        double dVelX = m_dDrivingSimulatorData[nTrial][t][DS_OWN_V];

        double dPrecedingPosX = m_dDrivingSimulatorData[nTrial][t][DS_PRECED_X];
        double dPrecedingVelX = m_dDrivingSimulatorData[nTrial][t][DS_PRECED_V];

		double dGapPreceding = dPrecedingPosX - dPosX;
        double dRelVelX = dPrecedingVelX - dVelX;

		int nProcess = -1;
		if (dPosY > dCenterlineY)
			nProcess = 1;
		else
			nProcess = 2;

		if (nProcess == 1 && (dPosX > dPrecedingPosX))
			nProcess = 3;

        int nClassificationResult = m_dDrivingSimulatorData[nTrial][t][DS_CLASS];

        if(FEATURE_VECTOR_DIMENSION == 2)
        {
            *out << t << "," << dPosX << "," << dPosY << "," << nProcess << "," << dGapPreceding << "," << dRelVelX << "," << "," << nClassificationResult << endl;
        }
        else if(FEATURE_VECTOR_DIMENSION == 3)
        {
            double dKdB = 0.0;
            double dK = 4.0 * 10000000.0 * dRelVelX / (dGapPreceding * dGapPreceding * dGapPreceding);

            if(dK < -1.0)
            {
                dKdB = 10 * log10(-dK);
            }
            else if(dK > 1.0)
            {
                dKdB = -10 * log10(dK);
            }
            else
            {
                dKdB = 0.0;
            }

            double dPotential = CPotential::GetInstance()->Field(dGapPreceding, dRelVelX, 180.0);
            dArrayPotential[t] = dPotential;

            double dAvg = 0.0;

            if(t >= 602)
            {
                double dSum = 0.0;

                for(int k=0; k<600; k++)
                {
                    dSum += dArrayPotential[t-k];
                }

                dAvg = dSum / 600.0;
            }

            if(dAvg != 0.0 && nProcess == 1 && t < 5000)
                *out << t << "," << dPosX << "," << dPosY << "," << nProcess << "," << "," << dGapPreceding << "," << dRelVelX << "," << dKdB << "," << "," << dPotential << ", " << dAvg << "," << "," << nClassificationResult << endl;
        }

		
	}

	fp->close();

	delete fp;
	delete out;

	return DONE;
}

void CDatabase::SaveDetectionResult( int nDriverNo )
{
    QString fileName = "../Log/detection_result";
    fileName += QString::number(nDriverNo) + ".csv";

    QFile* fp = new QFile(fileName);
    if (!fp->open(QIODevice::WriteOnly))
    {
        return;
    }

    QTextStream* out = new QTextStream(fp);

    for( int i=0; i<NUM_STATE; i++ )
    {
        for( int k=0; k<10; k++ )
        {
            int nTrialNo = 100 * nDriverNo + 10 * i + k;

            if( m_dDetectionResult[nTrialNo][0] == 0.0 )
                continue;

            *out << nTrialNo << "," << m_dDetectionResult[nTrialNo][0] << "," << "," << m_dDetectionResult[nTrialNo][1] << "," << m_dDetectionResult[nTrialNo][2] << "," << m_dDetectionResult[nTrialNo][3] << endl;
        }

        *out << endl;
    }

    fp->close();

    delete fp;
    delete out;
}

void CDatabase::SaveDetectionResult( void )
{
    QString fileName = "../Log/detection_result_all.csv";

    QFile* fp = new QFile(fileName);
    if (!fp->open(QIODevice::WriteOnly))
    {
        return;
    }

    QTextStream* out = new QTextStream(fp);

    for( int n=0; n<NUM_DRIVER; n++)
    {
        for( int i=0; i<NUM_STATE; i++ )
        {
            for( int k=0; k<10; k++ )
            {
                int nTrialNo = 100 * n + 10 * i + k;

                if( m_dDetectionResult[nTrialNo][0] == 0.0 )
                    continue;

                *out << nTrialNo << "," << m_dDetectionResult[nTrialNo][0] << "," << "," << m_dDetectionResult[nTrialNo][1] << "," << m_dDetectionResult[nTrialNo][2] << "," << m_dDetectionResult[nTrialNo][3] << endl;
            }

            *out << endl;
        }
    }

    fp->close();

    delete fp;
    delete out;
}

void CDatabase::SetDsParameterData( int nColumn, double dValue )
{
    m_dDsParameter[nColumn] = dValue;
}

double CDatabase::GetDsParameterData( int nColumn )
{
    return m_dDsParameter[nColumn];
}

void CDatabase::SetIntentionProbability(int nClass, double dValue)
{
    m_dIntentionProbability[nClass] = dValue;
}

double CDatabase::GetIntentionProbability(int nClass)
{
    return m_dIntentionProbability[nClass];
}

void CDatabase::SetEstimatedResult( int nTick, int nResult )
{
    m_nEstimatedResultData[nTick] = nResult;
}

int CDatabase::GetEstimatedResult( int nTick )
{
    return m_nEstimatedResultData[nTick];
}

void CDatabase::SetFeatureData(int nVehicleIndex, int nTime, int nColumn, double dValue)
{
    m_dFeatureData[nVehicleIndex][nTime][nColumn] = dValue;
}

double CDatabase::GetFeatureData(int nVehicleIndex, int nTime, int nColumn)
{
    return m_dFeatureData[nVehicleIndex][nTime][nColumn];
}

void CDatabase::SetGapAcceptanceProbability( double dValue )
{
    m_dAcceptanceProbability = dValue;
}

double CDatabase::GetGapAcceptanceProbability( void )
{
    return m_dAcceptanceProbability;
}

///////////////////////////////////////////////////////////////////////////
/* Private member functions */
///////////////////////////////////////////////////////////////////////////
int CDatabase::loadDSData(QString fileName)
{
	m_pOpenFile = new QFile(fileName);

	if (!m_pOpenFile->open(QIODevice::ReadOnly))
	{
		delete m_pOpenFile;
		qDebug() << "database.cpp @ Selected file does not exist.";
		return FAIL;
	}

	QTextStream in(m_pOpenFile);

	for (int i = 0; i < 7; i++)
	{
		QString line = in.readLine();
	}

	int t = 0;
	int nNumTrial = 0;
	while (!in.atEnd())
	{
		double dTemp[DS_NUM_COLUMN] = { 0.0 };
		QString line = in.readLine();
		QStringList list = line.split(',');

		for (int j = 0; j<list.size(); j++)
		{
			dTemp[j] = list.at(j).toDouble();
		}

		if (t == 0)
		{
			for (int j = 0; j < DS_NUM_COLUMN; j++)
				m_dDrivingSimulatorData[nNumTrial][t][j] = dTemp[j];

			t++;
		}
		else
		{
            double dPosX = dTemp[DS_OWN_X];
            double dPrePosX = m_dDrivingSimulatorData[nNumTrial][t - 1][DS_OWN_X];

			if (dPosX != dPrePosX)
			{
				if (dPosX < dPrePosX)
				{
					m_nDataInfo[nNumTrial][DATA_INFO_PACKET_DATA_LENGTH] = t;
					nNumTrial++;
					t = 0;

					continue;
				}

				for (int j = 0; j < DS_NUM_COLUMN; j++)
				{
					m_dDrivingSimulatorData[nNumTrial][t][j] = dTemp[j];
				}
					
				t++;
			}
		}

		if (t >= DS_T_MAX)
		{
			qDebug() << "database.cpp @ Driving simulator data length was overed max value";
			return FAIL;
		}
	}

	nNumTrial++;

	delete m_pOpenFile;
	m_nTrial = nNumTrial;

	return DONE;
}

int CDatabase::loadDSAllData(QString pathName, int nDriverNo, int nStateNo)
{
    int nReturn = FAIL;

    QString fileName;
    fileName = pathName + DRIVER[nDriverNo] + STATE[nStateNo] + ".csv";

    nReturn = loadDSData(fileName);

    if(nReturn)
    {
        QString file;
        file = DRIVER[nDriverNo] + STATE[nStateNo] + ".csv";
        qDebug() << "Database @ " + file + " is loaded..";

        int nTrial = GetNumTrial();
        qDebug() << "Database @ Num. of Trials: " + QString::number(nTrial);
    }

    return nReturn;
}

int CDatabase::loadDSThreeData(void)
{
    QString file[3] = {"training(cautious).csv", "training(normal).csv", "training(aggressive).csv"};
    int nNumTrial = 0;

    for(int f=0; f<3; f++)
    {
        QString fileName;
        fileName += DS_TRAININGFILE_PATH;
        fileName += file[f];

        m_pOpenFile = new QFile(fileName);

        if (!m_pOpenFile->open(QIODevice::ReadOnly))
        {
            delete m_pOpenFile;
            qDebug() << "database.cpp @ Selected file does not exist.";
            return FAIL;
        }

        QTextStream in(m_pOpenFile);

        for (int i = 0; i < 7; i++)
        {
            QString line = in.readLine();
        }

        int t = 0;

        while (!in.atEnd())
        {
            double dTemp[DS_NUM_COLUMN] = { 0.0 };
            QString line = in.readLine();
            QStringList list = line.split(',');

            for (int j = 0; j<list.size(); j++)
            {
                dTemp[j] = list.at(j).toDouble();
            }

            if (t == 0)
            {
                for (int j = 0; j < DS_NUM_COLUMN; j++)
                    m_dDrivingSimulatorData[nNumTrial][t][j] = dTemp[j];

                t++;
            }
            else
            {
                double dPosX = dTemp[DS_OWN_X];
                double dPrePosX = m_dDrivingSimulatorData[nNumTrial][t - 1][DS_OWN_X];

                if (dPosX != dPrePosX)
                {
                    if (dPosX < dPrePosX)
                    {
                        m_nDataInfo[nNumTrial][DATA_INFO_PACKET_DATA_LENGTH] = t;
                        nNumTrial++;
                        t = 0;

                        continue;
                    }

                    for (int j = 0; j < DS_NUM_COLUMN; j++)
                    {
                        m_dDrivingSimulatorData[nNumTrial][t][j] = dTemp[j];
                    }

                    t++;
                }
            }

            if (t >= DS_T_MAX)
            {
                qDebug() << "database.cpp @ Driving simulator data length was overed max value";
                return FAIL;
            }
        }

        nNumTrial++;

        delete m_pOpenFile;
    }

    m_nTrial = nNumTrial;

    return DONE;
}
