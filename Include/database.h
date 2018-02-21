#pragma once

#include "defines.h"

class QFile;
class QString;

/**
* @class	CDatabase
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2016/01/16 \n
* 			    Last revision date: 2016/01/16 HanwoolWoo
* @brief	this class is for saving measured data
*/
class CDatabase
{
public:
    static CDatabase* GetInstance()
    {
        static CDatabase* instance = new CDatabase();
        return instance;
    }

    ~CDatabase() {}

    /**
    * @fn Initialize
    * @param void
    * @return void
    * @remark this function is to initialize private members
    */
    void Initialize(void);

    /**
    * @fn LoadData
    * @param int data mode
    * @return int
    * @remark this function is to load the data such as the traffic data and the line points
    */
    int LoadData(int nDataMode, QString fileName);

    int LoadData(int nDataMode, QString fileName, int nDriverNo, int nStateNo);

    /**
    * @fn GetDataLength
    * @param void
    * @return int m_nDataLength
    * @remark this function is to get a data length of a traffic data
    */
    int GetDataLength(void);

    /**
    * @fn GetData
    * @param int nVehicleType, int nVehicleIndex, int nTime, int nColumn
    * @return double dValue
    * @remark this function is to get a data value that is saved in memory
    */
    double GetData(int nVehicleType, int nVehicleIndex, int nTime, int nColumn);

    /**
    * @fn SetFeatureData
    * @param int nVehicleIndex, int nTime, int nColumn, double dValue
    * @return void
    * @remark this function is to set a data value to the memory
    */
    void SetFeatureData(int nVehicleIndex, int nTime, int nColumn, double dValue);

    /**
    * @fn GetFeatureData
    * @param int nVehicleIndex, int nTime, int nColumn
    * @return double
    * @remark this function is to get a data value that is saved the memory
    */
    double GetFeatureData(int nVehicleIndex, int nTime, int nColumn);

    /**
    * @fn GetDataInfo
    * @param int nVehicleType, int nVehicleIndex, int nColumn
    * @return double dValue
    * @remark this function is to get a data value that is saved in memory
    */
    double GetDataInfo(int nVehicleType, int nVehicleIndex, int nColumn);

    /**
    * @fn GetNumAdjacentVehicles
    * @param void
    * @return int m_nNumAdjacentVehicles
    * @remark this function is to get the number of adjacent vehicles
    */
    int GetNumAdjacentVehicles( void );

    void SetAdjacentVehicleData( int nColumn, double dValue );
    double GetAdjacentVehicleData( int nColumn );

    void SetFlagShowOthers( bool flag );
    bool GetFlagShowOthers( void );

    void SetDrivingIntention( int nIntention);
    int GetDrivingIntention( void );

	void SetCollisionFlag(bool bFlag);
	bool GetCollisionFlag(void);

    void SetEstimatedResult( int nTick, int nResult );
    int GetEstimatedResult( int nTick );

	void SetDataInfo(int nVehicleIndex, int nColumn, int nValue);
	int GetDataInfo(int nVehicleIndex, int nColumn);

    void* GetDataPointer( int nDataType );

    void SetGroundTruth(int nValue);
	int GetGroundTruth(void);

	void SetPredictedTrajectory(int nIndex, double dPosX, double dPosY);
	void GetPredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY);

    void SetPrecedingTrajectory(int nIndex, double dPosX, double dPosY);
    void GetPrecedingTrajectory(int nIndex, double* pdPosX, double* pdPosY);

    void SetLeadTrajectory(int nIndex, double dPosX, double dPosY);
    void GetLeadTrajectory(int nIndex, double* pdPosX, double* pdPosY);

	int GetNumTrial(void);
	int GetCurrentTrial(void) { return m_nCurrentTrial; }
	void SetCurrentTrial(int nValue) { m_nCurrentTrial = nValue; }

    int SaveDSdataResult(int nTrial, int nDataLength);
    int SaveDSdataResult(int nDriverNo, int nState, int nTrial, int nDataLength);
    int SaveDataPerTrial(void);

	void SetDsClassificationResult(int nCurrentTrial, int nTick, int nResult);
    int GetDsClassificationResult(int nCurrentTrial, int nTick);

    void SetDsParameterData(int nTrial, int nTick, int nColumn, double dValue);
    double GetDsParameterData(int nTrial, int nTick, int nColumn);

    void SetIntentionProbability(int nClass, double dValue);
    double GetIntentionProbability(int nClass);

private:
    CDatabase();

    int saveFeatureData( void );

    int loadDSData(QString fileName);
    int loadDSAllData(QString fileName, int nDriverNo, int nStateNo);
    int loadDSThreeData(void);

    QFile* m_pOpenFile;

    int m_nDataLength;
    int m_nDataInfo[NUM_TRAFFIC_DATA][5]; // 0:vehicle no, 1:data length, 2:ground truth


    bool m_bShowOthers;
    int m_nDrivingIntention;
	bool m_bCollisionFlag;
    int m_nGroundTruth;
    double m_dDrivingSimulatorData[DS_NUM_TRIAL][DS_T_MAX][DS_NUM_COLUMN];
    double m_dDsParameterData[DS_NUM_TRIAL][DS_T_MAX][DS_NUM_COLUMN];
	int m_nTrial;
	int m_nCurrentTrial;

    double m_dPredictedTrajectory[100][2]; // [index][X,Y]
    double m_dPrecedingTrajectory[100][2]; // [index][X,Y]
    double m_dLeadTrajectory[100][2]; // [index][X,Y]
    double m_dIntentionProbability[NUM_CLASS];
    int m_nEstimatedResultData[DS_T_MAX];
    double m_dFeatureData[DS_NUM_TRIAL][DS_T_MAX][FEATURE_VECTOR_DIMENSION];
};
