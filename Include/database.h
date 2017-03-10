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

    void LoadModel( stHMM* model, int mode );

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

    /**
    * @fn SaveData
    * @param int nDataType
    * @return void
    * @remark this function is to save data to a file
    */
    void SaveData( int nDataType );

    void SetAdjacentVehicleData( int nColumn, double dValue );
    double GetAdjacentVehicleData( int nColumn );

    void SetFlagShowOthers( bool flag );
    bool GetFlagShowOthers( void );

    void SetLaneChangingFlag( bool flag );
    bool GetLaneChangingFlag( void );

	void SetCollisionFlag(bool bFlag);
	bool GetCollisionFlag(void);

    void SetEstimatedResult( int nTick, int nResult );
    int GetEstimatedResult( int nTick );

	void SetDataInfo(int nVehicleIndex, int nColumn, int nValue);
	int GetDataInfo(int nVehicleIndex, int nColumn);

    void* GetDataPointer( int nDataType );

	int GetGroundTruth(void);

	void SetPredictedTrajectory(int nIndex, double dPosX, double dPosY);
	void GetPredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY);

	void SetRePredictedTrajectory(int nIndex, double dPosX, double dPosY);
    void GetRePredictedTrajectory(int nIndex, double* pdPosX, double* pdPosY);

	int GetNumTrial(void);
	int GetCurrentTrial(void) { return m_nCurrentTrial; }
	void SetCurrentTrial(int nValue) { m_nCurrentTrial = nValue; }

    int SaveDSdataResult(int nTrial, int nDataLength);
    int SaveDataPerTrial(void);

	void SetDsClassificationResult(int nCurrentTrial, int nTick, int nResult);

    void SetDsParameterData(int nTrial, int nTick, int nColumn, double dValue);
    double GetDsParameterData(int nTrial, int nTick, int nColumn);

private:
    CDatabase();

    int saveFeatureData( void );

    int findAdjVehicles(int nVehicleNo, int nDataLength);

    int loadApproximateCurves( void );

	int findGroundTruth(int nDataLength);

	int loadTrafficData(int nDataMode, QString fileName);
    int loadDSData(QString fileName);

    QFile* m_pOpenFile;

    int m_nDataLength;
    int m_nDataInfo[NUM_TRAFFIC_DATA][5]; // 0:vehicle no, 1:data length, 2:ground truth
    int m_nAdjDataInfo[MAX_ADJACENT][10]; // 0:No 1:data length 2:vehicle length 3:vehicle width

    double m_dTargetVehicleData[T_MAX][NUM_COLUMN];

    int m_nNumAdjacentVehicles;
    double m_dAdjacentVehicleData[NUM_ADJ_VEHICLE][T_MAX][NUM_COLUMN];

    double m_dTrafficData[NUM_TRAFFIC_DATA][T_MAX][NUM_COLUMN];

    double m_dFeatureData[NUM_TRAFFIC_DATA][T_MAX][FEATURE_VECTOR_DIMENSION];
    int m_nEstimatedResultData[T_MAX];
    int m_nLabelData[NUM_TRAFFIC_DATA][T_MAX];

    // Coefficients of Approximate curves
    double m_dApproximateCurves[NUM_LINE][3];

    double m_dAdjDataForPotentialFeature[20];

    bool m_bShowOthers;
    bool m_LaneChangingFlag;
	bool m_bCollisionFlag;
	int m_nGroundTruth;

	double m_dPredictedTrajectory[(int)(TRAJECTORY_PREDICTION_TIME*10)][2];
	double m_dRePredictedTrajectory[(int)(TRAJECTORY_PREDICTION_TIME * 10)][2];

	double m_dDrivingSimulatorData[DS_NUM_TRIAL][DS_T_MAX][DS_NUM_COLUMN];
    double m_dDsParameterData[DS_NUM_TRIAL][DS_T_MAX][DS_NUM_COLUMN];
	int m_nTrial;
	int m_nCurrentTrial;
};
