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

CDatabase::CDatabase()
{
    Initialize();
}
///////////////////////////////////////////////////////////////////////////
/* Public functions */
///////////////////////////////////////////////////////////////////////////
/// \brief CDatabase::Initialize
///
void CDatabase::Initialize( void )
{
    m_nDataLength = 0;
    m_nNumAdjacentVehicles = 0;
	m_nGroundTruth = 0;

    memset( m_dTargetVehicleData, 0, sizeof( double ) * T_MAX * NUM_COLUMN );
    memset( m_dAdjacentVehicleData, 0, sizeof( double ) * NUM_ADJ_VEHICLE * T_MAX * NUM_COLUMN );
    memset( m_nAdjDataInfo, 0, sizeof(int) * MAX_ADJACENT * 10 );
    memset( m_dApproximateCurves, 0, sizeof(double) * NUM_LINE * 3 );

    memset( m_dTrafficData, 0, sizeof( double ) * NUM_TRAFFIC_DATA * T_MAX * NUM_COLUMN );
    memset( m_dFeatureData, 0, sizeof( double ) * NUM_TRAFFIC_DATA * T_MAX * FEATURE_VECTOR_DIMENSION );
    memset( m_nLabelData, 0, sizeof( int ) * NUM_TRAFFIC_DATA * T_MAX );
    memset( m_nEstimatedResultData, 0, sizeof( int ) * T_MAX );

	memset(m_dDrivingSimulatorData, 0, sizeof(double) * DS_NUM_TRIAL * DS_T_MAX * DS_NUM_COLUMN);
    memset(m_dDsParameterData, 0, sizeof(double) * DS_NUM_TRIAL * DS_T_MAX * DS_NUM_COLUMN);
	m_nTrial = 0;
	m_nCurrentTrial = 0;

    m_bShowOthers = true;
    m_LaneChangingFlag = false;
	m_bCollisionFlag = false;
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

void CDatabase::SetLaneChangingFlag( bool flag )
{
    m_LaneChangingFlag = flag;
}

bool CDatabase::GetLaneChangingFlag( void )
{
    return m_LaneChangingFlag;
}

void CDatabase::SetDataInfo(int nVehicleIndex, int nColumn, int nValue)
{
	m_nDataInfo[nVehicleIndex][nColumn] = nValue;
}

int CDatabase::GetDataInfo(int nVehicleIndex, int nColumn)
{
	return m_nDataInfo[nVehicleIndex][nColumn];
}

void CDatabase::SetPredictedTrajectory(int nIndex, double dPosX, double dPosY)
{
	m_dPredictedTrajectory[nIndex][0] = dPosX;
	m_dPredictedTrajectory[nIndex][1] = dPosY;
}

void CDatabase::GetPredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY)
{
	*pdPosX = m_dPredictedTrajectory[nIndex][0];
	*pdPosY = m_dPredictedTrajectory[nIndex][1];
}

void CDatabase::SetRePredictedTrajectory(int nIndex, double dPosX, double dPosY)
{
	m_dRePredictedTrajectory[nIndex][0] = dPosX;
	m_dRePredictedTrajectory[nIndex][1] = dPosY;
}

void CDatabase::GetRePredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY)
{
	*pdPosX = m_dRePredictedTrajectory[nIndex][0];
	*pdPosY = m_dRePredictedTrajectory[nIndex][1];
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
		nReturn = loadTrafficData(nDataMode, fileName);
		break;

	case DRIVING_SIMULATOR_DATA:
        nReturn = loadDSData(fileName);
		break;
	}

	return nReturn;
}

void CDatabase::LoadModel( stHMM* model, int mode )
{
    QString fileName;

    switch( mode )
    {
    case CHANGING:
        fileName = "Model/" + QString::number( STATE_NUM ) + " states_" + QString::number( FEATURE_VECTOR_DIMENSION ) + " dimension_changing_model.csv";
        break;

    case KEEPING:
        fileName = "Model/" + QString::number( STATE_NUM ) + " states_" + QString::number( FEATURE_VECTOR_DIMENSION ) + " dimension_keeping_model.csv";
        break;
    }

    m_pOpenFile = new QFile( fileName );

    if( !m_pOpenFile->open(QIODevice::ReadOnly) )
    {
        delete m_pOpenFile;
        return;
    }

    QTextStream in( m_pOpenFile );

    int t = 0;
    int k=0;
    int i=0;
    while( !in.atEnd() )
    {
        QString line = in.readLine();
        QStringList list = line.split(',');

        if( t == 1 )
            model->likelihood = list.at(0).toFloat();
        else if( t == 2 )
        {
            for( int j=0; j<list.size(); j++ )
                model->INIT[j] = list.at(j).toFloat();
        }
        else if( t >= 3 && t<(3+STATE_NUM) )
        {
            for( int j=0; j<list.size(); j++ )
                model->TRANS[t-3][j] = list.at(j).toFloat();
        }
        else if( t >= (3+STATE_NUM)  )
        {
            for( int j=0; j<list.size(); j++ )
                model->EMIS[i][k][j] = list.at(j).toFloat();

            k++;
            if( k == FEATURE_VECTOR_DIMENSION )
            {
                k=0;
                i++;
            }
        }

        t++;
    }

    m_pOpenFile->close();
    delete m_pOpenFile;
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
    case TARGET:
        return m_dTargetVehicleData[nTime][nColumn];

    case ADJACENT:
        return m_dAdjacentVehicleData[nVehicleIndex][nTime][nColumn];

	case DS:
		return m_dDrivingSimulatorData[nVehicleIndex][nTime][nColumn];
    }

    return FAIL;
}

void* CDatabase::GetDataPointer( int nDataType )
{
    switch( nDataType )
    {
    case FEATURE:
        return reinterpret_cast<void*>(m_dFeatureData);
    }

    return NULL;
}

///
/// \brief CDatabase::SetFeatureData
/// \param nVehicleIndex
/// \param nTime
/// \param nColumn
/// \param dValue
///
void CDatabase::SetFeatureData(int nVehicleIndex, int nTime, int nColumn, double dValue)
{
    m_dFeatureData[nVehicleIndex][nTime][nColumn] = dValue;
}

///
/// \brief CDatabase::GetFeatureData
/// \param nVehicleIndex
/// \param nTime
/// \param nColumn
/// \return
///
double CDatabase::GetFeatureData(int nVehicleIndex, int nTime, int nColumn)
{
    return m_dFeatureData[nVehicleIndex][nTime][nColumn];
}

double CDatabase::GetDataInfo(int nVehicleType, int nVehicleIndex, int nColumn)
{
    switch(nVehicleType)
    {
    case ADJACENT:
        return m_nAdjDataInfo[nVehicleIndex][nColumn];
    }

    return FAIL;
}

///
/// \brief CDatabase::GetNumAdjacentVehicles
/// \return
///
int CDatabase::GetNumAdjacentVehicles( void )
{
    return m_nNumAdjacentVehicles;
}

void CDatabase::SetAdjacentVehicleData(int nColumn, double dValue)
{
    m_dAdjDataForPotentialFeature[nColumn] = dValue;
}

double CDatabase::GetAdjacentVehicleData( int nColumn )
{
    return m_dAdjDataForPotentialFeature[nColumn];
}

void CDatabase::SaveData( int nDataType )
{
    switch( nDataType )
    {
    case FEATURE: saveFeatureData(); break;
    }
}

void CDatabase::SetEstimatedResult( int nTick, int nResult )
{
    m_nEstimatedResultData[nTick] = nResult;
}

int CDatabase::GetEstimatedResult( int nTick )
{
    return m_nEstimatedResultData[nTick];
}

int CDatabase::GetNumTrial(void)
{
	return m_nTrial;
}

void CDatabase::SetDsClassificationResult(int nCurrentTrial, int nTick, int nResult)
{
    m_dDrivingSimulatorData[nCurrentTrial][nTick][DS_CLASS] = (double)nResult;
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

        double dGain1 = m_dDsParameterData[nTrial][t][0];
        double dGain2 = m_dDsParameterData[nTrial][t][1];
        double dDampingRatio = m_dDsParameterData[nTrial][t][2];

		int nProcess = -1;
		if (dPosY > dCenterlineY)
			nProcess = 1;
		else
			nProcess = 2;

		if (nProcess == 1 && (dPosX > dPrecedingPosX))
			nProcess = 3;

		int nClassificationResult = m_dDrivingSimulatorData[nTrial][t][4];

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

void CDatabase::SetDsParameterData(int nTrial, int nTick, int nColumn, double dValue)
{
    m_dDsParameterData[nTrial][nTick][nColumn] = dValue;
}

double CDatabase::GetDsParameterData(int nTrial, int nTick, int nColumn)
{
    return m_dDsParameterData[nTrial][nTick][nColumn];
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

int CDatabase::loadTrafficData(int nDataMode, QString fileName)
{
	m_pOpenFile = new QFile(fileName);

	if (!m_pOpenFile->open(QIODevice::ReadOnly))
	{
		delete m_pOpenFile;
		qDebug() << "database.cpp @ Selected file does not exist.";
		return FAIL;
	}

	QTextStream in(m_pOpenFile);

	int t = 0;
	while (!in.atEnd())
	{
		QString line = in.readLine();
		QStringList list = line.split(',');

		for (int j = 0; j<list.size(); j++)
		{
			m_dTargetVehicleData[t][j] = list.at(j).toDouble();
		}

		t++;

		if (t >= T_MAX)
		{
			qDebug() << "database.cpp @ Traffic data length was overed max value";
			return FAIL;
		}
	}

	delete m_pOpenFile;

	int nVehicleNo = m_dTargetVehicleData[0][DATA_PACKET_NO];
	m_nDataLength = t;

	switch (nDataMode)
	{
	case TRAFFIC_DATA:
		// Print the number of loaded files
		qDebug() << "database.cpp @ Vehicle No." + QString::number(nVehicleNo) + " file was loaded...";
		break;

	case MEASUREMENT_DATA:
		return DONE; //break;
	}

	// Find adjacent vehicles using traffic data.
	int nNumAdjVehicles = findAdjVehicles(nVehicleNo, m_nDataLength);

	if (nNumAdjVehicles < 0)
	{
		qDebug() << "database.cpp @ Load adjacent data was failed...";
		return FAIL;
	}

	qDebug() << "database.cpp @ " << nNumAdjVehicles << " files were loaded as adjacent vehicles";

	// Load line information from files
	loadApproximateCurves();


	findGroundTruth(m_nDataLength);

	return DONE;
}

int CDatabase::findGroundTruth(int nDataLength)
{
	int nStartLane = m_dTargetVehicleData[0][DATA_PACKET_LANE];

	for (int t = 0; t < nDataLength; t++)
	{
		int nCurrentLane = m_dTargetVehicleData[t][DATA_PACKET_LANE];

		if (nStartLane != nCurrentLane)
		{
			m_nGroundTruth = t;
			return DONE;
		}
	}

	return FAIL;
}

int CDatabase::saveFeatureData( void )
{
    QString fileName = "../../Log/Feature/No";
    int nVehicleNo = m_dTargetVehicleData[0][DATA_PACKET_NO];
    fileName += QString::number( nVehicleNo ) + ".csv";

    QFile* fp = new QFile(fileName);
    if (!fp->open(QIODevice::WriteOnly))
    {
        return FAIL;
    }

    QTextStream* out = new QTextStream(fp);

    for (int t = 0; t<m_nDataLength; t++)
    {
        *out << t << ",";

        for (int k = 0; k < FEATURE_VECTOR_DIMENSION; k++)
        {
            *out << m_dFeatureData[TARGET][t][k] << ",";
        }
        *out << m_nEstimatedResultData[t] << endl;
    }

    fp->close();

    delete fp;
    delete out;

    qDebug() << "database.cpp @ Saving the feature data was completed";

    return DONE;
}

int CDatabase::findAdjVehicles(int nVehicleNo, int nDataLength)
{
    int nStartScene = m_dTargetVehicleData[0][DATA_PACKET_SCENE];
    int nEndScene = m_dTargetVehicleData[nDataLength-1][DATA_PACKET_SCENE];

    int nSearchingStartNo = nVehicleNo - 200;
    if( nSearchingStartNo < 0 ) nSearchingStartNo = 0;

    int nSearchingEndNo = nVehicleNo + 200;
    if( nSearchingEndNo > MAX_VEHICLE_NO ) nSearchingEndNo = MAX_VEHICLE_NO;

    QString path;
    QFile* pOpenFile;

    path = TRAFFIC_FILE_PATH;
    path += "test_vehicle/";

    for( int num=nSearchingStartNo; num<nSearchingEndNo; num++ )
    {
        QString file = path + "no" + QString::number( num ) + ".csv";

        pOpenFile = new QFile( file );

        if( !pOpenFile->open(QIODevice::ReadOnly) )
        {
            delete pOpenFile;
            continue;
        }

        QTextStream in( pOpenFile );

        int nTime = 0;
        double dTemp[T_MAX][NUM_COLUMN] = { 0.0 };
        while( !in.atEnd() )
        {
            QString line = in.readLine();
            QStringList list = line.split(',');

            for( int j=0; j<list.size(); j++ )
            {
                dTemp[nTime][j] = list.at(j).toDouble();
            }

            nTime++;

            if( nTime >= T_MAX )
            {
                qDebug() << "database.cpp @ adjacent vehicle data length wad over";
                return FAIL;
            }
        }

		// Conduct a filtering
		//CFilter::GetInstance()->MovingAverageFilter(nTime, reinterpret_cast<void*>(dTemp));

        bool flag = false;
        int nSceneGap = nStartScene - dTemp[0][DATA_PACKET_SCENE];

        for( int t=0; t<nTime; t++ )
        {
            if( dTemp[t][DATA_PACKET_SCENE] >= nStartScene && dTemp[t][DATA_PACKET_SCENE] <= nEndScene )
            {
                flag = true;
                break;
            }
        }

        if( flag == false )
        {
            delete pOpenFile;
            continue;
        }
        flag = false;

        //
        // Check distance with respect to the target vehicle
        //
        if( nSceneGap >= 0 )
        {
            int k=0;
            for( int t=nSceneGap; t<nTime; t++ )
            {
                int nScene1 = dTemp[t][DATA_PACKET_SCENE];
                int nScene2 = m_dTargetVehicleData[k][DATA_PACKET_SCENE];

                if( nScene1 != nScene2 )
                {
                    qDebug() << "database.cpp @ Error : Scene is not synchronized,  no." << dTemp[0][0];
                    delete pOpenFile;
                    return FAIL;
                }

                int nLane1 = dTemp[t][DATA_PACKET_LANE];
                int nLane2 = m_dTargetVehicleData[k][DATA_PACKET_LANE];
                int nDelta = qAbs( nLane1 - nLane2 );

                double dPosY1 = dTemp[t][DATA_PACKET_Y];
                double dPosY2 = m_dTargetVehicleData[k][DATA_PACKET_Y];
                double dGap = qAbs( dPosY1 - dPosY2 );

                if( (nDelta==1) && (dGap * FEET_TO_METER < SENSING_AREA) ) //! distance limit is 50 m
                {
                    flag = true;
                    break;
                }

                k++;

                if( k >= nDataLength )
                    break;
            }
        }
        else
        {
            int k=-nSceneGap;
            for( int t=0; t<nTime; t++ )
            {
                int nScene1 = dTemp[t][DATA_PACKET_SCENE];
                int nScene2 = m_dTargetVehicleData[k][DATA_PACKET_SCENE];

                if( nScene1 != nScene2 )
                {
                    qDebug() << "database.cpp @ Error : Scene is not synchronized,  no." << dTemp[0][0];
                    delete pOpenFile;
                    return FAIL;
                }

                int nLane1 = dTemp[t][DATA_PACKET_LANE];
                int nLane2 = m_dTargetVehicleData[k][DATA_PACKET_LANE];
                int nDelta = qAbs( nLane1 - nLane2 );

                double dPosY1 = dTemp[t][DATA_PACKET_Y];
                double dPosY2 = m_dTargetVehicleData[k][DATA_PACKET_Y];
                double dGap = qAbs( dPosY1 - dPosY2 );

                if( (nDelta==1) && (dGap*FEET_TO_METER < SENSING_AREA) ) //! distance limit is 50 m
                {
                    flag = true;
                    break;
                }

                k++;

                if( k >= nDataLength )
                    break;
            }
        }

        if( flag == false )
        {
            delete pOpenFile;
            continue;
        }

        if( nSceneGap >= 0 )
        {
            int k=0;
            for( int t=nSceneGap; t<nTime; t++ )
            {
                int nScene1 = dTemp[t][DATA_PACKET_SCENE];
                int nScene2 = m_dTargetVehicleData[k][DATA_PACKET_SCENE];

                if( nScene1 != nScene2 )
                {
                    qDebug() << "database.cpp @ Error 2 : Scene is not synchronized,  no." << dTemp[0][0];
                    delete pOpenFile;
                    return FAIL;
                }

                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_VEHICLE_NO] = dTemp[0][DATA_PACKET_NO];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_SCENE] = dTemp[t][DATA_PACKET_SCENE];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_X] = dTemp[t][DATA_PACKET_X];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_Y] = dTemp[t][DATA_PACKET_Y];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_LANE] = dTemp[t][DATA_PACKET_LANE];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][k][ADJ_DATA_PACKET_VEL] = dTemp[t][DATA_PACKET_VEL];

                k++;

                if( k >= nDataLength )
                    break;
            }

            for( int i=k; i<nDataLength; i++ )
            {
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_VEHICLE_NO] = dTemp[0][DATA_PACKET_NO];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_SCENE] = -1;
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_X] = -1;
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_Y] = -1;
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_LANE] = -1;
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][i][ADJ_DATA_PACKET_VEL] = -1;
            }
        }
        else
        {
            int k=nSceneGap;
            for( int t=0; t<nDataLength; t++ )
            {
                if( k < 0 || k >= nTime )
                {
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_VEHICLE_NO] = dTemp[0][DATA_PACKET_NO];
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_SCENE] = -1;
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_X] = -1;
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_Y] = -1;
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_LANE] = -1;
                    m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_VEL] = -1;

                    k++;
                    continue;
                }

                int nScene1 = dTemp[k][DATA_PACKET_SCENE];
                int nScene2 = m_dTargetVehicleData[t][DATA_PACKET_SCENE];

                if( nScene1 != nScene2 )
                {
                    qDebug() << "Error 2 : Scene is not synchronized,  no." << dTemp[0][0];
                    delete pOpenFile;
                    return FAIL;
                }

                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_VEHICLE_NO] = dTemp[0][DATA_PACKET_NO];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_SCENE] = dTemp[k][DATA_PACKET_SCENE];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_X] = dTemp[k][DATA_PACKET_X];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_Y] = dTemp[k][DATA_PACKET_Y];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_LANE] = dTemp[k][DATA_PACKET_LANE];
                m_dAdjacentVehicleData[m_nNumAdjacentVehicles][t][ADJ_DATA_PACKET_VEL] = dTemp[k][DATA_PACKET_VEL];

                k++;
            }
        }

        m_nAdjDataInfo[m_nNumAdjacentVehicles][0] = dTemp[0][DATA_PACKET_NO];
        m_nAdjDataInfo[m_nNumAdjacentVehicles][1] = nTime;
        m_nAdjDataInfo[m_nNumAdjacentVehicles][2] = dTemp[0][DATA_PACKET_LENGTH];
        m_nAdjDataInfo[m_nNumAdjacentVehicles][3] = dTemp[0][DATA_PACKET_WIDTH];

        m_nNumAdjacentVehicles++;

        delete pOpenFile;
    }

    return m_nNumAdjacentVehicles;
}

int CDatabase::loadApproximateCurves( void )
{
    QString file = "Lane/i80_approximateCurve.csv";

    m_pOpenFile = new QFile( file );

    if( !m_pOpenFile->open(QIODevice::ReadOnly) )
    {
        delete m_pOpenFile;
        return FAIL;
    }

    QTextStream in( m_pOpenFile );
    int nLine = 0;

    while( !in.atEnd() )
    {
        QString line = in.readLine();
        QStringList list = line.split(',');

        for( int j=0; j<list.size(); j++ )
        {
            m_dApproximateCurves[nLine][j] = list.at(j).toDouble();
        }

        nLine++;
        if( nLine > NUM_LINE ) return FAIL;
    }

    //CLineExtractor::GetInstance()->Set( m_dApproximateCurves );

    delete m_pOpenFile;
    return DONE;
}

















