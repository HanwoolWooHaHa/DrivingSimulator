/**
* @file	extractor.h
* @version	1.00
* @author	Hanwool Woo
* @date	Creation date: 2016/07/23
* @brief	this file works to extract features using traffic data or measurement data
*/

#pragma once

class CExtractor
{
public:
    static CExtractor* GetInstance()
    {
        static CExtractor* instance = new CExtractor();
        return instance;
    }

    ~CExtractor();

    void Initialize(void);

    void Extract( int nTick );
    void ChangeSide( int nSide );

private:
    CExtractor();


    double calculatePotentialValue( int nTick );
    double calcDistanceToCenterline(int nTick);
    double calcLateralVelocity(int nTick);

    double m_dAdjDataForPotentialFeature[20];

    int m_nSide;
};
