#pragma once

/**
* @file	define.h
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/07/21
* @brief	this file defines constant values
*/

#define TRAFFIC
//#define MEASUREMENT
#define DRVING_SIMULATOR

#define BASIC
//#define MIKAMI
//#define FILEV
//#define PROPOSED


#define TRAFFIC_FILE_PATH "../../Dataset/i-80/traffic/"
#define DS_FILE_PATH "../../Dataset/mazda/20160826_DS/"

#define DONE 1
#define FAIL -1

// 1. Data mode
#define DATA_MODE_DEFAULT 0
#define TRAFFIC_DATA 1
#define MEASUREMENT_DATA 2
#define TRAFFIC_DATA_AUTO 3
#define DRIVING_SIMULATOR_DATA 4

enum eData{ RAW=0, FEATURE, LABEL };
enum eVEHICLE{ PRIMARY=0, TARGET, ADJACENT, LINE, FUTURE, DS };
enum {LEAD_VEHICLE_NO = 0, LEAD_VEHICLE_GAP, LEAD_VEHICLE_REL_VEL, REAR_VEHICLE_NO, REAR_VEHICLE_GAP, REAR_VEHICLE_REL_VEL,
      PRECEDING_VEHICLE_NO, PRECEDING_VEHICLE_GAP, PRECEDING_VEHICLE_REL_VEL,
      FOLLOWING_VEHICLE_NO, FOLLOWING_VEHICLE_GAP, FOLLOWING_VEHICLE_REL_VEL };
enum eLANE_CHANGING_PROCESS{ KEEPING = 0, CHANGING, ARRIVAL, ADJUSTMENT };

#if defined(TRAFFIC)
// DATA
#define NUM_LINE_FILES 250
#define NUM_LINE 6
#define MAX_LINE_PT 500
#define MAX_VEHICLE_NO 4000
#define T_MAX 3000
#define NUM_COLUMN 18

#define NUM_TRAFFIC_DATA 400
#define NUM_ADJ_VEHICLE 800

#define MAX_Y 3000 // feet
#define RESOLUTION_LINE_PINT 0.328084
#define SENSING_AREA 100 // meter
#define LANE_WIDTH 12.28

// Traffic data packet
#define DATA_PACKET_NO 0
#define DATA_PACKET_SCENE 1
#define DATA_PACKET_LABEL 2
#define DATA_PACKET_X 4
#define DATA_PACKET_Y 5
#define DATA_PACKET_LENGTH 8
#define DATA_PACKET_WIDTH 9
#define DATA_PACKET_VEL 11
#define DATA_PACKET_LANE 13
#define DATA_PACKET_PRECEDING_VEH 14
#define DATA_PACKET_FOLLOWING_VEH 15

// DATA INFO PACKET
#define DATA_INFO_PACKET_VEHICLE_NO 0
#define DATA_INFO_PACKET_DATA_LENGTH 1
#define DATA_INFO_PACKET_GROUND_TRUTH 2
#define DATA_INFO_PACKET_START_LANE 3
#define DATA_INFO_PACKET_END_LANE 4

// Filter
#define MOV_AVG_SPAN 25 // must be set the odd number
#define MED_FLT_SPAN 7
#define NUM_STATE 4 // used by Bayesian filter
#define NUM_CLASS 4

#if defined (KUMAR)
    #define BAYESIAN_FILTER
#endif

// FEATURE VECTOR
#if defined (PROPOSED)
    #define FEATURE_VECTOR_DIMENSION 3
#elif defined(MANDALIA)
    #define FEATURE_VECTOR_DIMENSION 4
#elif defined(SCHLECHTRIEMEN)
    #define FEATURE_VECTOR_DIMENSION 3
#elif defined(PROPOSED_PF)
    #define FEATURE_VECTOR_DIMENSION 3
#elif defined(MIKAMI)
    #define FEATURE_VECTOR_DIMENSION 3
#else
    #define FEATURE_VECTOR_DIMENSION 2
#endif
#define FEATURE_PACKET_DISTANCE 0
#define FEATURE_PACKET_LAT_VELOCITY 1
#define FEATURE_PACKET_POTENTIAL 2
#define FEATURE_PACKET_LEAD_GAP 2
#define FEATURE_PACKET_LEAD_REL_VEL 3
#define FEATURE_PACKET_LAG_GAP 4
#define FEATURE_PACKET_LAG_REL_VEL 5
#define FEATURE_PACKET_LEAD_PROB_GAP 2
#define FEATURE_PACKET_LAG_PROB_GAP 3
#define FEATURE_PACKET_PROB_GAP 2
#define FEATURE_PACKET_REL_VELOCITY 2

// LABEL
#define LANE_KEEPING 1
#define LANE_CHANGING 2
#define LANE_ADJUSTMENT 3

#define METER_TO_FEET 3.28
#define FEET_TO_METER 0.3048

// ADJACENT VEHICLE DATA
#define NUM_ADJACENT_COLUMN 6
#define MAX_ADJACENT 200
#define ADJ_DATA_PACKET_VEHICLE_NO 0
#define ADJ_DATA_PACKET_SCENE 1
#define ADJ_DATA_PACKET_X 2
#define ADJ_DATA_PACKET_Y 3
#define ADJ_DATA_PACKET_LANE 4
#define ADJ_DATA_PACKET_VEL 5

// HIDDEN MARKOV MODEL (HMM)
#define STATE_NUM 4
#define STD_DISTANCE 1.87
#define STD_VELOCITY 1.22
#define STD_REL_VELOCITY 8.3

// MANDALIA
#define MANDALIA_WINDOW_SIZE 12

// WINDOW SIZE for SVM
#if defined (MANDALIA)
    #define WINDOW_SIZE 1
#elif (SCHLECHTRIEMEN)
    #define WINDOW_SIZE 1
#else
    #define WINDOW_SIZE 3
#endif

// ESTIMATION TIME LIMIT
#define ESTIMATION_TIME_LIMIT 66

// TRAJECTORY PREDICTION TIME
#define TRAJECTORY_PREDICTION_TIME 1.0

#elif defined(MEASUREMENT)
// 1. Real Data packet
#define PACK_TIME 0
#define PACKET_OWN_GLOBAL_X 1 // Longitude
#define PACKET_OWN_GLOBAL_Y 2 // Latitude
#define PACKET_OWN_GLOBAL_ATT 3 //Attitude angle

// 2. Data file value
#define T_MAX 2000
#define FILE_COLUMN_MAX 26
#define MDATA_COLUMN_MAX 40
#define LINE_MAX_INDEX 26475
#define RECORD_COLUMN 10
#define NUM_LANE 4

// 3. Sensors setting
//#define LASER_SENS_AREA 80.0 // meter

// 4. Environment setting
#define LANE_WIDTH 3.5

// 5. Approximation setting
#define GAUSS_N 3
#define SENSING_RANGE 100.0
#define RESOLUTION_LINE_POINT 0.1

// 6. Measurement data version
#define ALL_VEHICLE 1
//#define SELECTED_VEHICLE 2
#define ADJ_VEH_MAX 30

// 7. Feature vector info
#define FEATURE_VECTOR_DIMENSION 3

#define FEATURE_PACKET_DISTANCE 0
#define FEATURE_PACKET_LAT_VELOCITY 1
#define FEATURE_PACKET_POTENTIAL 2

#define ADJ_AMOUNTS_LEAD_IDX 0
#define ADJ_AMOUNTS_LEAD_GAP 1
#define ADJ_AMOUNTS_LEAD_VEL 2
#define ADJ_AMOUNTS_REAR_IDX 3
#define ADJ_AMOUNTS_REAR_GAP 4
#define ADJ_AMOUNTS_REAR_VEL 5
#define ADJ_AMOUNTS_PRECED_IDX 6
#define ADJ_AMOUNTS_PRECED_GAP 7
#define ADJ_AMOUNTS_PRECED_VEL 8
#define ADJ_AMOUNTS_OWN_IDX 9
#define ADJ_AMOUNTS_OWN_GAP 10
#define ADJ_AMOUNTS_OWN_VEL 11

// 8. Normalization
#define STD_DISTANCE (0.5 * LANE_WIDTH) // [m]
#define STD_LATERAL_VELOCITY 1.22 // [m/s]


//  9. Hidden Markov Model (HMM)
#define NUM_STATE 4


//  10. Support Vector Machine
#define NUM_CLASS 4
#define WINDOW_SIZE 1

//  11. Estimation result packet
#define RESULT_PACKET_NUM 4
#define RESULT_PACKET_FLAG 0
#define RESULT_PACKET_CLASS 1

//  12. Filter definition
#define MOV_AVG_FLT_SPAN 25 // must be set the odd number
#define MED_FLT_SPAN 7
#endif

#define DS_NUM_TRIAL 48
#define DS_T_MAX 20000
#define DS_NUM_COLUMN 44

#define DS_OWN_X 1 // Training:1, Test:4
#define DS_OWN_Y 5
#define DS_OWN_V 6
#define DS_PRECED_X 7
#define DS_PRECED_Y 8
#define DS_PRECED_V 9
#define DS_LEAD_X 10
#define DS_LEAD_Y 11
#define DS_LEAD_V 12

#define DS_FEATURE_VEL 5
#define DS_FEATURE_DST 4
#define DS_FEATURE_PE 8
#define DS_CLASS 10 // Training:10, Test 3

#define DS_FEATURE_VEL_MEAN -0.9981
#define DS_FEATURE_DST_MEAN 45.59765
#define DS_FEATURE_PE_MEAN 0.290535
#define DS_FEATURE_RISK_MEAN 6.891076

#define DS_FEATURE_VEL_STD 2.410828
#define DS_FEATURE_DST_STD 19.7844
#define DS_FEATURE_PE_STD 0.20633
#define DS_FEATURE_RISK_STD 25.21421

struct stHMM
{
    float INIT[NUM_STATE];
    float TRANS[NUM_STATE][NUM_STATE];
    float EMIS[NUM_STATE][FEATURE_VECTOR_DIMENSION][2];
    double likelihood;
};
