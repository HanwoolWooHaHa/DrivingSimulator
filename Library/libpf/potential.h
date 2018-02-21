#pragma once

/**
* @class	CPotential
* @author	Hanwool Woo
* @version	1.10
* @date	Creation date: 2016/02/09 \n
* 			    Last revision date: 2016/02/09 HanwoolWoo
* @brief	this class is for the generation of potential field
*/
class CPotential
{
public:
	static CPotential* GetInstance()
    {
        static CPotential* instance = new CPotential();
        return instance;
    }

    ~CPotential();

    double Field( double dDistance, double dVelocity, int nDir );
	double Field( double dDistance, double dVelocity, double dAngle );

private:
	CPotential();


	double COEFFICIENT;
};
